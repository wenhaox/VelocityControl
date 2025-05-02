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

// Structure for actuator parameters
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

struct BrakeParams {
    int    channel;        // Analog output channel driving the brake
    int    offset;         // Amps→bits offset (from XML)
    double scale;          // Amps→bits scale  (from XML)
    double releaseCurrent; // Current during 0.5 s release pulse
    double holdCurrent;    // Current that keeps brake released
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

// Constant array for PID configuration
static const PIDParams kPIDParams[] = {
    { 0, "yaw", "REVOLUTE",   600.0,  30.0, 0.0, true,  0.01, 50.0 },
    { 1, "pitch", "REVOLUTE", 600.0,  30.0, 0.0, true,  0.01, 50.0 },
    { 2, "insertion", "PRISMATIC", 6000.0, 200.0, 0.0, true, 0.005, 20.0 },
    { 3, "disc_1", "REVOLUTE",    1,   0.08, 0.0, true, 0.0005, 10.0 },
    { 4, "disc_2", "REVOLUTE",    1,   0.08, 0.0, true, 0.0005, 10.0 },
    { 5, "disc_3", "REVOLUTE",    1,   0.08, 0.0, true, 0.0005, 10.0 },
    { 6, "disc_4", "REVOLUTE",    1,   0.08, 0.0, true, 0.0005, 10.0 }
};

static const BrakeParams kBrakeTable[] = {
    /* axis‑0 */ {7,  32768, 16000.0,  0.15,  0.08},
    /* axis‑1 */ {8,  32768, 16000.0,  0.15,  0.08},
    /* axis‑2 */ {9, -32768, 16000.0, -0.25, -0.25}
};

constexpr double kBrakeReleaseTime = 0.5;   // seconds

// Reads the actuator configuration from the JSON file
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
        // Load the lookup table. Expecting exactly 4096 values
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
 
// Converts a raw encoder count to SI units with corrected 24-bit wrap-around
double convertEncoderPosition(int32_t encoderCount, const ActuatorParams &params) {
    // Center the count and handle wrap-around for a 24-bit encoder
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
 
// Reverse conversion: from SI units to encoder count with wrapped 24-bit range
int reverseConvertToEncoderCount(double siUnits, const ActuatorParams &params) {
    double conversionFactor = params.isPrismatic ? mm2m : deg2rad;
    int count = static_cast<int>(siUnits / (params.encoderScale * conversionFactor));
    count = count + 0x800000;
    
    // Wrap the count into the valid 24-bit range (0 to 2^24 - 1)
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

inline int BrakeAmpsToBits(const BrakeParams &bp, double amps)
{
    return static_cast<int>(bp.offset + bp.scale * amps);
}

static void releaseBrake(const BrakeParams &bp, BasePort *port, AmpIO *board)
{
    //Pulse the high release current
    int bits = BrakeAmpsToBits(bp, bp.releaseCurrent);
    board->SetMotorCurrent(bp.channel, bits);
    port->WriteAllBoards();
    Amp1394_Sleep(kBrakeReleaseTime);

    //Switch to the lower hold current
    bits = BrakeAmpsToBits(bp, bp.holdCurrent);
    board->SetMotorCurrent(bp.channel, bits);
    port->WriteAllBoards();
}

static void engageBrakes(BasePort *port, AmpIO *board)
{
    for (const auto &bp : kBrakeTable)
        board->SetMotorCurrent(bp.channel, BrakeAmpsToBits(bp, 0.0));
    port->WriteAllBoards();
}

//---------------------------------------------------------------------
//  This function is called repeatedly to set the desired velocity
//---------------------------------------------------------------------
bool setVelocity(double                         desiredVelocity,
    int                            selectedMotor,
    BasePort*                      Port,
    std::vector<AmpIO*>&           BoardList,
    const std::vector<ActuatorParams>& actuators,
    double&                        prevTime,
    double&                        prevError)
{
    // ---------- 0. Sanity checks ------------------------------------------------
    if (selectedMotor < 1 || selectedMotor > static_cast<int>(actuators.size())) {
        std::cerr << "Invalid motor index: " << selectedMotor << std::endl;
        return false;
    }
    const int pidIdx = selectedMotor - 1;

    // ---------- 1. PID gains ----------------------------------------------------
    double Kp = 0.0, Kd = 0.0;
    if (pidIdx < static_cast<int>(std::size(kPIDParams))) {
        Kp = kPIDParams[pidIdx].p_gain;
        Kd = kPIDParams[pidIdx].d_gain;
    }

    // ---------- 2. Acquire latest board state ----------------------------------
    if (!Port->ReadAllBoards()) {
        std::cerr << "Failed to read board status." << std::endl;
        return false;
    }
    Amp1394_Sleep(0.0005);

    // Map “selectedMotor” (1‑based) → board motor index (0‑based)
    const unsigned int motorIdx = actuators[pidIdx].index - 1;
    BoardList[0]->WriteMotorControlMode(motorIdx, AmpIO::CURRENT);
    Port->WriteAllBoards();

    const ActuatorParams& act = actuators[pidIdx];

    // ---------- 3. Measure velocity --------------------------------------------
    int32_t rawVel   = BoardList[0]->GetEncoderVelocityPredicted(motorIdx);
    double  conv     = act.isPrismatic ? mm2m : deg2rad;
    double  velSI    = static_cast<double>(rawVel) * act.encoderScale * conv;

    // ---------- 4. PID error terms ---------------------------------------------
    const double error = desiredVelocity - velSI;
    const double now   = Amp1394_GetTime();
    const double dt    = now - prevTime;

    // Derivative (clamped *before* multiplication)
    double dRaw = (error - prevError) / dt;
    dRaw        = std::clamp(dRaw, -50.0, 50.0);          //  A/s
    double dTerm = std::clamp(Kd * dRaw, -0.02, 0.02);    //  Nm or N

    // Proportional
    double pTerm = Kp * error;


    //----------------------------------------------------------
    // 4b.  Release brakes if on axes 0‑2
    //----------------------------------------------------------
    const ActuatorParams &act = actuators[selectedMotor - 1];
    if (act.index <= 3) {
        releaseBrake(kBrakeTable[act.index - 1], Port, BoardList[0]);
    }

    // ---------- 5. Combine + clamp + slew‑rate ---------------------------------
    double cmd   = std::clamp(pTerm + dTerm, -0.5, 0.5);  //  I_max = 0.5 A

    static double lastCmd = 0.0;
    const  double maxStep = 10.0 * dt;                    // slew 10 A/s
    cmd = std::clamp(cmd - lastCmd, -maxStep, maxStep) + lastCmd;
    lastCmd = cmd;

    // ---------- 6. Soft‑limit zone safety --------------------------------------
    int32_t rawPos = BoardList[0]->GetEncoderPosition(motorIdx);
    double  posSI  = convertEncoderPosition(rawPos, act);

    if (act.hasLimits) {
        const double range   = act.upperLimit - act.lowerLimit;
        const double margin  = range * 0.10;

        if (posSI <= act.lowerLimit + margin ||
            posSI >= act.upperLimit - margin) {
            cmd      = 0.0;
            lastCmd  = 0.0;
            BoardList[0]->SetMotorCurrent(motorIdx, convertCurrentToBits(0.0));
            Port->WriteAllBoards();
            return false;  // exit control loop
        }
    }

    // ---------- 7. Ship the command --------------------------------------------
    BoardList[0]->SetMotorCurrent(motorIdx, convertCurrentToBits(cmd));
    Port->WriteAllBoards();

    // ---------- 8. Debug print --------------------------------------------------
    std::cout << "Actuator " << act.index
    << ": DesiredVel=" << desiredVelocity
    << ", MeasVel="     << velSI
    << ", Err="         << error
    << ", dRaw="        << dRaw
    << ", P="           << pTerm
    << ", D="           << dTerm
    << ", CMD="         << cmd
    << std::endl;

    // ---------- 9. Update history ----------------------------------------------
    prevError = error;
    prevTime  = now;
    return true;
}

//---------------------------------------------------------------------
//  Helper that encapsulates the main control loop.
//  This function is called from main() to start the control loop.
//---------------------------------------------------------------------
int runControl(const std::string &configFile,
    int               selectedMotor,
    double            desiredVelocity)
{
    //----------------------------------------------------------
    // 1.  Parse actuator configuration
    //----------------------------------------------------------
    std::vector<ActuatorParams> actuators = readActuatorConfigs(configFile);
    printActuatorInfo(actuators);

    //----------------------------------------------------------
    // 2.  Hardware initialisation
    //----------------------------------------------------------
    std::vector<AmpIO*> BoardList;
    std::string         portDescription = BasePort::DefaultPort();
    std::stringstream   debugStream;
    BasePort*           Port = PortFactory(portDescription.c_str(), debugStream);

    if (!Port || !Port->IsOK()) {
        std::cerr << "Port init failed." << std::endl;
        return -1;
    }

    unsigned char discoveredID = Port->GetBoardId(0);
    BoardList.emplace_back(new AmpIO(discoveredID));
    Port->AddBoard(BoardList[0]);

    if (!Port->SetProtocol(BasePort::PROTOCOL_SEQ_RW)) {
        std::cerr << "Protocol set failed." << std::endl;
        return -1;
    }
    if (Port->GetNumOfNodes() == 0) {
        std::cerr << "No nodes found." << std::endl;
        return -1;
    }

    //----------------------------------------------------------
    // 3.  Board power‑up
    //----------------------------------------------------------
    BoardList[0]->WriteSafetyRelay(true);
    BoardList[0]->WritePowerEnable(true);
    BoardList[0]->WriteAmpEnableAxis(selectedMotor - 1, true);

    //----------------------------------------------------------
    // 4.  Encoder pre‑load from pots
    //----------------------------------------------------------
    for (size_t i = 0; i < actuators.size(); ++i) {
        unsigned int motorIdx   = actuators[i].index - 1;
        int          analogVal  = BoardList[0]->GetAnalogInput(motorIdx);
        double       siPreset   = 0.0;

    if (!actuators[i].lookupTable.empty() &&
        analogVal >= 0 && analogVal < 4096) {
        siPreset = actuators[i].lookupTable[analogVal];
    } else {
        std::cerr << "Lookup table not available for actuator "
                << actuators[i].index << std::endl;
    }

    int preload = reverseConvertToEncoderCount(siPreset, actuators[i]);
    BoardList[0]->WriteEncoderPreload(motorIdx, preload);

    std::cout << "Actuator " << actuators[i].index
        << ": Analog="          << analogVal
        << ", SI Preset="       << siPreset
        << ", Encoder Preload=0x" << std::hex << preload
        << std::dec << std::endl;
    }

    //----------------------------------------------------------
    // 5.  Real‑time control loop
    //----------------------------------------------------------
    double prevTime  = Amp1394_GetTime();
    double prevError = 0.0;

    while (true) {
        bool ok = setVelocity(desiredVelocity,
                        selectedMotor,
                        Port,
                        BoardList,
                        actuators,
                        prevTime,
                        prevError);
        if (!ok) {
            std::cout << "Exiting control loop (limit reached)." << std::endl;
            break;
    }

        std::cout << "Control command sent at time: "
            << std::fixed << std::setprecision(6)
            << Amp1394_GetTime() << " s" << std::endl;

        Amp1394_Sleep(0.01);   // 10 ms
    }

    //----------------------------------------------------------
    // 6.  Shutdown (same sequence as original code)
    //----------------------------------------------------------
    BoardList[0]->WritePowerEnable(false);
    BoardList[0]->WriteAmpEnable(0x0f, 0x00);
    BoardList[0]->WriteSafetyRelay(false);
    engageBrakes(Port, BoardList[0]);

    Port->RemoveBoard(BoardList[0]->GetBoardId());
    delete BoardList[0];
    delete Port;
    return 0;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config.json>" << std::endl;
        return -1;
    }

    const std::string configFile      = argv[1];
    constexpr int     kSelectedMotor  = 7;    // 1‑based index
    constexpr double  kDesiredVelSI   = 0.5;  // rad/s or m/s

    return runControl(configFile, kSelectedMotor, kDesiredVelSI);
}
