#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>
#include <iomanip>
#include <map>
#include <utility>
#include "jsoncpp/include/json/json.h"
#include <ZynqEmioPort.h>
#include <Amp1394/AmpIORevision.h>
#include "PortFactory.h"
#include "AmpIO.h"
#include "Amp1394Time.h"
#include "Amp1394Console.h"
#include "EthBasePort.h"

// Structure for actuator parameters, note the added lookupTable member.
struct ActuatorParams {
    int index;
    double encoderScale;         // Scale from raw encoder bits to physical units (per bit)
    std::string jointType;       // "REVOLUTE" or "PRISMATIC"
    bool isPrismatic;            // true if prismatic
    bool hasLimits;              // true if soft limits are specified
    double lowerLimit;           // Soft limit in SI
    double upperLimit;           // Soft limit in SI
    std::vector<double> lookupTable; // 4096-entry lookup table for converting analog input to SI preset
    ActuatorParams() : index(0), encoderScale(0.0), jointType("UNKNOWN"),
                       isPrismatic(false), hasLimits(false), lowerLimit(0.0), upperLimit(0.0) {}
};

struct PIDParams {
    int index;
    std::string name;
    std::string type;
    double p_gain;
    double d_gain;
    double i_gain;
    bool use_disturbance_observer;
    double nominal_mass;
    double disturbance_cutoff;
};

// Unit conversion factors
const double deg2rad = M_PI / 180.0;
const double mm2m   = 0.001;

// Constant array for PID configuration.
static const PIDParams kPIDParams[] = {
    { 0, "yaw", "REVOLUTE",   600.0,  30.0, 0.0, true,  0.01, 50.0 },
    { 1, "pitch", "REVOLUTE", 600.0,  30.0, 0.0, true,  0.01, 50.0 },
    { 2, "insertion", "PRISMATIC", 6000.0, 200.0, 0.0, true, 0.005, 20.0 },
    { 3, "disc_1", "REVOLUTE",    1,   0.08, 0.0, true, 0.0005, 10.0 },
    { 4, "disc_2", "REVOLUTE",    1,   0.08, 0.0, true, 0.0005, 10.0 },
    { 5, "disc_3", "REVOLUTE",    1,   0.08, 0.0, true, 0.0005, 10.0 },
    { 6, "disc_4", "REVOLUTE",    1,   0.08, 0.0, true, 0.0005, 10.0 }
};

// Reads the actuator configuration from the JSON file.
std::vector<ActuatorParams> readActuatorConfigs(const std::string &configFile) {
    std::vector<ActuatorParams> actuators;
    std::ifstream inConfig(configFile);
    if (!inConfig.is_open()) {
        std::cerr << "Failed to open configuration file: " << configFile << std::endl;
        exit(EXIT_FAILURE);
    }
    Json::Value robot_json;
    Json::Reader reader;
    if (!reader.parse(inConfig, robot_json)) {
        std::cerr << "Failed to parse JSON configuration:" << std::endl;
        std::cerr << reader.getFormattedErrorMessages() << std::endl;
        exit(EXIT_FAILURE);
    }
    Json::Value actuatorConfigs = robot_json["Robots"][0]["Actuators"];
    if (actuatorConfigs.empty()) {
        std::cerr << "No actuators found in configuration file." << std::endl;
        exit(EXIT_FAILURE);
    }
    for (unsigned int i = 0; i < actuatorConfigs.size(); i++) {
        ActuatorParams actuator;
        actuator.index = i + 1;
        actuator.encoderScale = actuatorConfigs[i]["Encoder"]["BitsToPosition"]["Scale"].asDouble();
        actuator.jointType = actuatorConfigs[i]["JointType"].asString();
        actuator.isPrismatic = (actuator.jointType == "PRISMATIC");
        if (actuatorConfigs[i]["Encoder"].isMember("PositionLimitsSoft")) {
            Json::Value limitsSoft = actuatorConfigs[i]["Encoder"]["PositionLimitsSoft"];
            actuator.hasLimits = true;
            double lower = limitsSoft["Lower"].asDouble();
            double upper = limitsSoft["Upper"].asDouble();
            std::string unit = limitsSoft["Unit"].asString();
            if (!actuator.isPrismatic) {
                if (unit == "deg") {
                    actuator.lowerLimit = lower * deg2rad;
                    actuator.upperLimit = upper * deg2rad;
                } else {
                    actuator.lowerLimit = lower;
                    actuator.upperLimit = upper;
                }
            } else {
                if (unit == "mm") {
                    actuator.lowerLimit = lower * mm2m;
                    actuator.upperLimit = upper * mm2m;
                } else {
                    actuator.lowerLimit = lower;
                    actuator.upperLimit = upper;
                }
            }
        }
        // Load the lookup table if it exists. Expecting exactly 4096 values.
        if (actuatorConfigs[i].isMember("Pot") &&
        actuatorConfigs[i]["Pot"].isMember("LookupTable"))
        {
        Json::Value lookup = actuatorConfigs[i]["Pot"]["LookupTable"];
        if (lookup.size() == 4096) {
            for (unsigned int j = 0; j < lookup.size(); j++) {
                actuator.lookupTable.push_back(lookup[j].asDouble());
            }
        } else {
            std::cerr << "Warning: Expected 4096 lookup table entries for actuator " 
                      << actuator.index << ", found " << lookup.size() << std::endl;
            }
        }
        actuators.push_back(actuator);
    }
    inConfig.close();
    return actuators;
}
 
// Converts a raw encoder count to SI units with corrected 24-bit wrap-around.
double convertEncoderPosition(int32_t encoderCount, const ActuatorParams &params) {
    // Center the count and handle wrap-around for a 24-bit encoder.
    int32_t diff = encoderCount - 0x800000;
    const int32_t encoderMax = 0x1000000; // 2^24

    if (diff > encoderMax / 2)
        diff -= encoderMax;
    else if (diff <= -static_cast<int>(encoderMax / 2))
        diff += encoderMax;

    double posUnits = static_cast<double>(diff) * params.encoderScale;
    double conversionFactor = params.isPrismatic ? mm2m : deg2rad;
    return posUnits * conversionFactor;
}
 
// Reverse conversion: from SI units to encoder count with wrapped 24-bit range.
int reverseConvertToEncoderCount(double siUnits, const ActuatorParams &params) {
    double conversionFactor = params.isPrismatic ? mm2m : deg2rad;
    int count = static_cast<int>(siUnits / (params.encoderScale * conversionFactor));
    count = count + 0x800000;
    
    // Wrap the count into the valid 24-bit range (0 to 2^24 - 1).
    const int encoderMax = 0x1000000; // 2^24
    if (count < 0)
        count += encoderMax;
    else if (count >= encoderMax)
        count -= encoderMax;
    
    return count;
}

int convertCurrentToBits(double controlSignal) {
    constexpr int offset = 32768;
    constexpr double scale = -4800.0; 
    int bits = static_cast<int>(offset + scale * controlSignal);
    return bits;
}
 
// Prints a summary of the actuator configurations.
void printActuatorInfo(const std::vector<ActuatorParams> &actuators) {
    std::cout << "Actuator Config Summary:" << std::endl;
    std::cout << std::setw(5) << "Idx"
              << std::setw(15) << "JointType"
              << std::setw(15) << "EncScale"
              << std::setw(10) << "Limits"
              << std::setw(20) << "Lower (SI)"
              << std::setw(20) << "Upper (SI)" << std::endl;
    for (const auto &actuator : actuators) {
        std::cout << std::setw(5) << actuator.index
                  << std::setw(15) << actuator.jointType
                  << std::setw(15) << std::fixed << std::setprecision(8) << actuator.encoderScale
                  << std::setw(10) << (actuator.hasLimits ? "Yes" : "No");
        if (actuator.hasLimits) {
            std::cout << std::setw(20) << actuator.lowerLimit
                      << std::setw(20) << actuator.upperLimit;
        } else {
            std::cout << std::setw(20) << "N/A" << std::setw(20) << "N/A";
        }
        std::cout << std::endl;
    }
}

bool setVelocity(double desiredVelocity,
    int selectedMotor,
    BasePort* Port,
    std::vector<AmpIO*>& BoardList,
    const std::vector<ActuatorParams>& actuators,
    double &prevTime,
    double &prevError)
{
    // Check that the selected motor is valid.
    if (selectedMotor < 1 || selectedMotor > (int)actuators.size()) {
    std::cerr << "Invalid motor index: " << selectedMotor << std::endl;
    return false;
    }

    // Look up PID gains from the constant array.
    int pidIndex = selectedMotor - 1;  
    double Kp = 0.0, Kd = 0.0;
    if (pidIndex < (int)(sizeof(kPIDParams)/sizeof(PIDParams))) {
    Kp = kPIDParams[pidIndex].p_gain;
    Kd = kPIDParams[pidIndex].d_gain;
    }

    // Read the board(s) and sleep briefly.
    if (!Port->ReadAllBoards()) {
    std::cerr << "Failed to read board status." << std::endl;
    return false;
    }
    Amp1394_Sleep(0.0005);

    // Get the actuator corresponding to the specified motor.
    unsigned int motorIdx = actuators[selectedMotor - 1].index - 1;
    BoardList[0]->WriteMotorControlMode(motorIdx, AmpIO::CURRENT);
    Port->WriteAllBoards();

    const ActuatorParams &act = actuators[selectedMotor - 1];

    // Get predicted velocity (counts/sec) and convert to SI units.
    int32_t rawVel = BoardList[0]->GetEncoderVelocityPredicted(motorIdx);
    double convFactor = (act.isPrismatic ? mm2m : deg2rad);
    double velSI = static_cast<double>(rawVel) * act.encoderScale * convFactor;

    // Compute error (desired velocity minus measured velocity).
    double error = desiredVelocity - velSI;

    // Compute dt.
    double currentTime = Amp1394_GetTime();
    double dt          = currentTime - prevTime;

    // 1) Derivative term 
    double rawD = (error - prevError)/dt;
    constexpr double Dmax = 50; // A/sec
    rawD = std::clamp(rawD, -Dmax, +Dmax);
    double dTerm = Kd * rawD;

    // Clamp dTerm after scaling
    constexpr double dTermMax = 0.02; 
    dTerm = std::clamp(dTerm, -dTermMax, +dTermMax);


    // 2) P-term
    double pTerm = Kp * error;

    // 3) Combine & clamp
    double rawU = pTerm + dTerm;
    constexpr double Imax = 0.5; // A
    double u = std::clamp(rawU, -Imax, +Imax);

    // 4) Slew-rate limiter
    static double lastU = 0.0;
    constexpr double slewRate = 10.0; // A/sec
    double maxStep = slewRate * dt;
    double delta   = std::clamp(u - lastU, -maxStep, +maxStep);
    u = lastU + delta;
    lastU = u;

    // Safety zone based on position.
    int32_t rawPos = BoardList[0]->GetEncoderPosition(motorIdx);
    double posSI  = convertEncoderPosition(rawPos, act);
    if (act.hasLimits) {
    double range        = act.upperLimit - act.lowerLimit;
    double safetyMargin = range * 0.10;
    if (posSI <= (act.lowerLimit + safetyMargin) ||
    posSI >= (act.upperLimit - safetyMargin)) {
    // Safety condition: send zero current and exit.
    u = 0.0;
    lastU = u;
    int bits = convertCurrentToBits(u);
    BoardList[0]->SetMotorCurrent(motorIdx, bits);
    Port->WriteAllBoards();
    return false;
    }
    }

    // Write the computed current.
    int bits = convertCurrentToBits(u);
    BoardList[0]->SetMotorCurrent(motorIdx, bits);
    Port->WriteAllBoards();

    // Debug output.
    std::cout << "Actuator " << act.index
    << ": Err=" << error
    << ", dRaw=" << rawD
    << ", P=" << pTerm
    << ", D=" << dTerm
    << ", CMD" << u << std::endl;

    // Update history.
    prevError = error;
    prevTime  = currentTime;
    return true;
}


int main(int argc, char** argv) {
    // Expect configuration file as argument.
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config.json>" << std::endl;
        return -1;
    }
    std::string configFile = argv[1];
    std::vector<ActuatorParams> actuators = readActuatorConfigs(configFile);
    printActuatorInfo(actuators);

    // Hardware initialization.
    std::vector<AmpIO*> BoardList;
    std::string portDescription = BasePort::DefaultPort();
    std::stringstream debugStream;
    BasePort* Port = PortFactory(portDescription.c_str(), debugStream);
    if (!Port || !Port->IsOK()) {
        std::cerr << "Port init failed." << std::endl;
        return -1;
    }

    // Add board and set protocol.
    unsigned char discoveredID = Port->GetBoardId(0);
    BoardList.push_back(new AmpIO(discoveredID));
    Port->AddBoard(BoardList[0]);
    if (!Port->SetProtocol(BasePort::PROTOCOL_SEQ_RW)) {
        std::cerr << "Protocol set failed." << std::endl;
        return -1;
    }

    // Check if the board is connected.
    if (Port->GetNumOfNodes() == 0) {
        std::cerr << "No nodes found." << std::endl;
        return -1;
    }

    // Set initial motor currents.
    const unsigned int MAX_AXES = 11;
    uint32_t MotorCurrents[MAX_AXES];
    for (unsigned int i = 0; i < MAX_AXES; i++)
        MotorCurrents[i] = 0x8000;

    // ------------------------------------------------------------------
    // Configure which motor to drive and the desired velocity (SI units).
    int selectedMotor = 7;      // control motor 1
    double desiredVelocity = 0.5; // Example desired velocity in SI units

    // Initialize board power and enable signals.
    BoardList[0]->WriteSafetyRelay(true);
    BoardList[0]->WritePowerEnable(true);
    BoardList[0]->WriteAmpEnableAxis(6, true);

    // Set encoder preloads from analog inputs.
    for (size_t i = 0; i < actuators.size(); i++) {
        unsigned int motorIdx = actuators[i].index - 1;
        int analogValue = BoardList[0]->GetAnalogInput(motorIdx);
        double siPreset = 0.0;
        if (!actuators[i].lookupTable.empty() && analogValue >= 0 && analogValue < 4096) {
            siPreset = actuators[i].lookupTable[analogValue];
        } else {
            std::cerr << "Lookup table not available for actuator "
                      << actuators[i].index << std::endl;
        }
        int preload = reverseConvertToEncoderCount(siPreset, actuators[i]);
        BoardList[0]->WriteEncoderPreload(motorIdx, preload);
        std::cout << "Actuator " << actuators[i].index
                  << ": Analog = " << analogValue
                  << ", SI Preset = " << siPreset
                  << ", Encoder Preload = 0x" << std::hex << preload << std::dec << std::endl;
    }

    // ** NEW: initialize timing & error history **
    double prevTime  = Amp1394_GetTime();
    double prevError = 0.0;

    // PD control loop: continuously update control command.
    while (true) {
        bool continueControl = setVelocity(desiredVelocity,
                                           selectedMotor,
                                           Port,
                                           BoardList,
                                           actuators,
                                           prevTime,
                                           prevError);
        if (!continueControl) {
            std::cout << "Exiting control loop, limit reached." << std::endl;
            break;
        }

        double currentTime = Amp1394_GetTime();
        std::cout << "Control command sent at time: " << std::fixed
                  << std::setprecision(6) << currentTime << " s" << std::endl;

        Amp1394_Sleep(0.01); // Sleep for 10ms between updates
    }

    // Shutdown sequence.
    BoardList[0]->WritePowerEnable(false);
    BoardList[0]->WriteAmpEnable(0x0f, 0x00);
    BoardList[0]->WriteSafetyRelay(false);
    Port->RemoveBoard(BoardList[0]->GetBoardId());
    delete BoardList[0];
    delete Port;
    return 0;
}