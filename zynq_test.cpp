#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>
#include <iomanip> // for std::setprecision
#include "jsoncpp/include/json/json.h" // Requires jsoncpp

#include <Amp1394/AmpIORevision.h>
#include "PortFactory.h"
#include "AmpIO.h"
#include "Amp1394Time.h"
#include "Amp1394Console.h"
#include "EthBasePort.h"

//TODO: Joint limit(PID config file for joint limit), 
// Create safety zone near limits (10% of range or fixed value) stop if within 5% of limit
// test joint limit wiht hand first and see for a printout
// get the feedback from the board, do the control computation, and then write the motor currents.  
// The first is done by calling ReadAllBoards in the port class (in your case, ZynqEmioPort).  
// Once you get the feedback, you would call AmpIO::GetEncoderVelocityPredicted to get the velocity (in counts/sec).  
// For writing the motor current, you would call AmpIO::SetMotorCurrent to build the local buffer, and then ZynqEmioPort::WriteAllBoards to actually send it to the FPGA. 

// osaUnitToSI Factors
const double deg2rad = M_PI / 180.0;
const double mm2m = 0.001;

// Structure to hold actuator parameters
struct ActuatorParams {
    int index;
    double encoderScale;
    std::string jointType;
    bool isPrismatic;
    bool hasLimits;
    double lowerLimit;
    double upperLimit;
    
    // Default constructor
    ActuatorParams() : index(0), encoderScale(0.0), jointType("UNKNOWN"),
                      isPrismatic(false), hasLimits(false),
                      lowerLimit(0.0), upperLimit(0.0) {}
};

// Read configuration for all actuators from the JSON file
std::vector<ActuatorParams> readActuatorConfigs(const std::string &configFile) {
    std::vector<ActuatorParams> actuators;
    std::ifstream inConfig(configFile);
    
    if (!inConfig.is_open()) {
        std::cerr << "Failed to open configuration file: " << configFile << std::endl;
        exit(EXIT_FAILURE);
    }

    Json::Value robot_json;
    Json::Reader reader;
    bool parsingSuccessful = reader.parse(inConfig, robot_json);
    if (!parsingSuccessful) {
        std::cerr << "Failed to parse JSON configuration." << std::endl;
        std::cerr << reader.getFormattedErrorMessages() << std::endl;
        exit(EXIT_FAILURE);
    }
    
    Json::Value actuatorConfigs = robot_json["Robots"][0]["Actuators"];
    if (actuatorConfigs.empty()) {
        std::cerr << "No actuators found in configuration file." << std::endl;
        exit(EXIT_FAILURE);
    }
    
    // Process each actuator
    for (unsigned int i = 0; i < actuatorConfigs.size(); i++) {
        ActuatorParams actuator;
        actuator.index = i + 1; // 1-indexed
        
        // Extract encoder parameters
        actuator.encoderScale = actuatorConfigs[i]["Encoder"]["BitsToPosition"]["Scale"].asDouble();
        
        // Check if joint is prismatic or revolute
        actuator.jointType = actuatorConfigs[i]["JointType"].asString();
        actuator.isPrismatic = (actuator.jointType == "PRISMATIC");
        
        // Check if joint limits are specified
        if (actuatorConfigs[i].isMember("PositionLimits")) {
            actuator.hasLimits = true;
            double lowerPos = actuatorConfigs[i]["PositionLimits"]["Lower"].asDouble();
            double upperPos = actuatorConfigs[i]["PositionLimits"]["Upper"].asDouble();
            
            // Apply the unit conversion to limits (raw units to SI)
            double conversionFactor = actuator.isPrismatic ? mm2m : deg2rad;
            actuator.lowerLimit = lowerPos * conversionFactor;
            actuator.upperLimit = upperPos * conversionFactor;
        }
        
        actuators.push_back(actuator);
    }

    inConfig.close();
    return actuators;
}

// Convert encoder position to SI units
double convertEncoderPosition(int32_t encoderCount, const ActuatorParams &params) {
    // Apply the offset of 0x800000 (midpoint)
    double positionBits = static_cast<double>(encoderCount - 0x800000);
    
    // Apply scale factor from configuration
    double positionUnits = positionBits * params.encoderScale;
    
    // Apply unit conversion based on joint type (deg to rad, or mm to m)
    double conversionFactor = params.isPrismatic ? mm2m : deg2rad;
    double positionSI = positionUnits * conversionFactor;
    
    return positionSI;
}

// Check if a position is within limits
bool checkPositionLimits(double positionSI, const ActuatorParams &params, bool printWarning = true) {
    if (!params.hasLimits) {
        if (printWarning) {
            std::cout << "Actuator " << params.index << " has no defined limits." << std::endl;
        }
        return true; // No limits defined, so considered "within limits"
    }
    
    if (positionSI < params.lowerLimit) {
        if (printWarning) {
            std::cout << "Actuator " << params.index << " position (" << positionSI 
                      << ") is below lower limit (" << params.lowerLimit << ")." << std::endl;
        }
        return false;
    }
    
    if (positionSI > params.upperLimit) {
        if (printWarning) {
            std::cout << "Actuator " << params.index << " position (" << positionSI 
                      << ") is above upper limit (" << params.upperLimit << ")." << std::endl;
        }
        return false;
    }
    
    return true;
}

// Print information about all actuators
void printActuatorInfo(const std::vector<ActuatorParams> &actuators) {
    std::cout << "Actuator Configuration Summary:" << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;
    std::cout << std::setw(5) << "Index" << std::setw(15) << "Joint Type" 
              << std::setw(15) << "Encoder Scale" << std::setw(15) << "Has Limits" 
              << std::setw(15) << "Lower Limit" << std::setw(15) << "Upper Limit" << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;
    
    for (const auto &actuator : actuators) {
        std::cout << std::setw(5) << actuator.index
                  << std::setw(15) << actuator.jointType
                  << std::setw(15) << std::fixed << std::setprecision(8) << actuator.encoderScale
                  << std::setw(15) << (actuator.hasLimits ? "Yes" : "No");
        
        if (actuator.hasLimits) {
            std::cout << std::setw(15) << std::fixed << std::setprecision(6) << actuator.lowerLimit
                      << std::setw(15) << std::fixed << std::setprecision(6) << actuator.upperLimit;
        } else {
            std::cout << std::setw(15) << "N/A" << std::setw(15) << "N/A";
        }
        std::cout << std::endl;
    }
    std::cout << "-------------------------------------------------" << std::endl;
}

// ---------------------------------------------------------------------------
// Main function with full setup, PD control loop, and shutdown
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    // Check for configuration file argument
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config.json>" << std::endl;
        std::cerr << "  - config.json: Path to the robot configuration file" << std::endl;
        return -1;
    }
    std::string configFile = argv[1];
    
    // Read actuator configurations from JSON
    std::vector<ActuatorParams> actuators = readActuatorConfigs(configFile);
    printActuatorInfo(actuators);
    
    // -------------------------------------------------------------------------
    // SETUP: Create and configure the port and boards (same as in qladisp)
    // -------------------------------------------------------------------------
    unsigned int numDisp = 0;      // Number of boards to display; assume one board for this example.
    std::vector<AmpIO*> BoardList;
    // For simplicity, select board 0 (adjust as needed)
    BoardList.push_back(new AmpIO(0));
    numDisp = 1;
    
    std::string portDescription = BasePort::DefaultPort();
    std::stringstream debugStream(std::stringstream::out | std::stringstream::in);
    BasePort::AddHardwareVersionStringList("");
    
    BasePort *Port = PortFactory(portDescription.c_str(), debugStream);
    if (!Port) {
        std::cerr << "Failed to create port using: " << portDescription << std::endl;
        return -1;
    }
    if (!Port->IsOK()) {
        std::cerr << "Failed to initialize " << Port->GetPortTypeString() << std::endl;
        return -1;
    }
    
    // Add board(s) to port
    for (size_t i = 0; i < BoardList.size(); i++) {
        Port->AddBoard(BoardList[i]);
    }
    
    // Set protocol (using default protocol)
    if (!Port->SetProtocol(BasePort::PROTOCOL_SEQ_RW))
        std::cerr << "Failed to set protocol. Using current protocol." << std::endl;
    
    // Initialize motor currents at mid-range (0x8000)
    const unsigned int MAX_AXES = 11;
    uint32_t MotorCurrents[MAX_AXES];
    for (unsigned int i = 0; i < MAX_AXES; i++) {
        MotorCurrents[i] = 0x8000;
    }
    
    // Power up the board(s)
    for (unsigned int j = 0; j < numDisp; j++) {
        BoardList[j]->WriteSafetyRelay(true);
        BoardList[j]->WritePowerEnable(true);
        // Enable all motors (assume 0x0f mask for 4 motors; adjust as needed)
        BoardList[j]->WriteAmpEnable(0x0f, 0x0f);
        // (Optionally) Write initial encoder preload if required
    }
    
    // Initialize console for key input and display (as in qladisp)
    Amp1394Console console;
    console.Init();
    if (!console.IsOK()) {
        std::cerr << "Failed to initialize console" << std::endl;
        return -1;
    }
    console.Print(1, 5, "Zynq PD Control Test");
    console.Print(2, 5, "Press ESC to quit");
    console.Refresh();
    
    // -------------------------------------------------------------------------
    // PD Control Loop
    // -------------------------------------------------------------------------
    // Create vector for previous errors (one per actuator)
    std::vector<double> prevError(actuators.size(), 0.0);
    
    // Set PD Gains (adjust experimentally)
    const double Kp = 100.0;
    const double Kd = 10.0;
    
    // Time management: record loop start time
    double startTime = Amp1394_GetTime();
    double lastTime = startTime;
    
    const int ESC_CHAR = 0x1b;
    int c;
    while ((c = console.GetChar()) != ESC_CHAR) {
        // Read feedback from boards
        Port->ReadAllBoards();
        
        double currentTime = Amp1394_GetTime();
        double dt = (currentTime - lastTime);
        if (dt <= 0.0) dt = 0.001; // safeguard
        lastTime = currentTime;
        
        // Loop through each actuator to compute PD control
        for (size_t i = 0; i < actuators.size(); i++) {
            // For this example, we assume each actuator is driven by board 0 and motor index = actuator.index - 1
            unsigned int motorIdx = actuators[i].index - 1;
            // Get encoder position and predicted velocity (in raw counts and counts/sec)
            int32_t rawPos = BoardList[0]->GetEncoderPosition(motorIdx);
            uint32_t velPred = BoardList[0]->GetEncoderVelocityPredicted(motorIdx);
            
            // Convert position to SI units using configuration function
            double posSI = convertEncoderPosition(rawPos, actuators[i]);
            // Convert predicted velocity to SI using appropriate conversion factor
            double conversionFactor = actuators[i].isPrismatic ? mm2m : deg2rad;
            double velSI = static_cast<double>(velPred) * conversionFactor;
            
            // Desired velocity is zero (hold position)
            double desiredVel = 0.0;
            double error = desiredVel - velSI;
            double derivative = (error - prevError[i]) / dt;
            prevError[i] = error;
            double controlEffort = Kp * error + Kd * derivative;
            
            // Safety zone: if position is near a limit, cap the control effort to zero.
            // Define safety (10% of range) and stop zone (5% of range)
            if (actuators[i].hasLimits) {
                double range = actuators[i].upperLimit - actuators[i].lowerLimit;
                double stopZone = 0.05 * range; // 5% of range
                if (posSI <= (actuators[i].lowerLimit + stopZone) ||
                    posSI >= (actuators[i].upperLimit - stopZone)) {
                    controlEffort = 0.0;
                    console.Print(4, 5, "Actuator %d safety zone active", actuators[i].index);
                }
            }
            
            // Compute commanded motor current (offset from mid-range 0x8000)
            uint32_t commandedCurrent = 0x8000 + static_cast<uint32_t>(controlEffort);
            // Set motor current in the local buffer
            BoardList[0]->SetMotorCurrent(motorIdx, commandedCurrent);
        }
        
        // Write updated motor currents to FPGA
        Port->WriteAllBoards();
        
        // Refresh console and sleep briefly (for 500 microseconds if not Zynq EMIO)
        console.Refresh();
        if (Port->GetPortType() != BasePort::PORT_ZYNQ_EMIO)
            Amp1394_Sleep(0.0005);
    }
    
    // -------------------------------------------------------------------------
    // SHUTDOWN: Turn off power and reset settings (exactly as in qladisp)
    // -------------------------------------------------------------------------
    for (unsigned int j = 0; j < numDisp; j++) {
        BoardList[j]->WritePowerEnable(false);      // Turn power off
        BoardList[j]->WriteAmpEnable(0x0f, 0x00);       // Disable all motors
        BoardList[j]->WriteSafetyRelay(false);
    }
    // (Optionally, reset encoder preloads to default if needed)
    
    console.End();
    // Remove boards from port and free resources
    for (size_t j = 0; j < BoardList.size(); j++) {
        Port->RemoveBoard(BoardList[j]->GetBoardId());
        delete BoardList[j];
    }
    delete Port;
    
    return 0;
}