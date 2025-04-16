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

// Unit conversion factors
const double deg2rad = M_PI / 180.0;
const double mm2m   = 0.001;

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
 
// Converts a raw encoder count to SI units.
double convertEncoderPosition(int32_t encoderCount, const ActuatorParams &params) {
    // Center the count and handle wrap-around for a 24-bit encoder
    int32_t diff = encoderCount - 0x800000;
    const int32_t encoderMax = 0x1000000; // 2^24
    if (diff > encoderMax / 2)
        diff -= encoderMax;
    else if (diff < -static_cast<int>(encoderMax / 2))
        diff += encoderMax;

    double posUnits = static_cast<double>(diff) * params.encoderScale;
    double conversionFactor = params.isPrismatic ? mm2m : deg2rad;
    return posUnits * conversionFactor;
}
 
// Reverse conversion: from SI units to encoder count using encoder scale and proper conversion factor.
int reverseConvertToEncoderCount(double siUnits, const ActuatorParams &params) {
    double conversionFactor = params.isPrismatic ? mm2m : deg2rad;
    int count = static_cast<int>(siUnits / (params.encoderScale * conversionFactor));
    count = count + 0x800000;
    
    // Wrap the count to the valid 24-bit range (0 to 2^24-1).
    const int encoderMax = 0x1000000; // 2^24
    if (count < 0)
        count += encoderMax;
    else if (count >= encoderMax)
        count -= encoderMax;
    
    return count;
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
    BasePort::AddHardwareVersionStringList("");
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
 
    // Initialize board power and enable signals.
    BoardList[0]->WriteSafetyRelay(true);
    BoardList[0]->WritePowerEnable(true);
    BoardList[0]->WriteAmpEnable(0x0f, 0x0f);
 
   
    // ------------------------------------------------------------------
 
    std::string input;
    double lastTime = Amp1394_GetTime();
    while (true) {
        bool ret = Port->ReadAllBoards();
        Amp1394_Sleep(0.0005);
         // ----- Set encoder preset using analog input and lookup table -----
        // For each actuator, obtain the analog input value (0-4095) then use the lookup table
        // to get the SI preset value. Reverse‑convert to encoder count and set that as the preload.

        for (size_t i = 0; i < actuators.size(); i++) {
            unsigned int motorIdx = actuators[i].index - 1;
            int analogValue = BoardList[0]->GetAnalogInput(motorIdx); 
            double siPreset = 0.0;
            if (!actuators[i].lookupTable.empty() && analogValue >= 0 && analogValue < 4096) {
                siPreset = actuators[i].lookupTable[analogValue];
            } else {
                std::cerr << "Lookup table not available for actuator " << actuators[i].index << std::endl;
                // Fall back to a default preset, e.g., 0.
            }
            int preload = reverseConvertToEncoderCount(siPreset, actuators[i]);
            // Set encoder preload for this motor.
            BoardList[0]->WriteEncoderPreload(motorIdx, preload);
            std::cout << "Actuator " << actuators[i].index << ": Analog = " << analogValue 
                    << ", SI Preset = " << siPreset << ", Encoder Preload = 0x" 
                    << std::hex << preload << std::dec << std::endl;
        }
        double currentTime = Amp1394_GetTime();
        double deltaTime = currentTime - lastTime;
 
        std::cout << "\033[2J\033[H";  // Clear screen and reposition cursor
        std::cout << "=== Status at time: " << std::fixed << std::setprecision(6) << currentTime
                  << " (dt: " << deltaTime << " s) ===\n" << std::endl;
 
        uint32_t boardStatus = BoardList[0]->GetStatus();
        std::cout << "Board Status: 0x" << std::hex << std::setw(8) << std::setfill('0') << boardStatus
                  << std::dec << " | ReadAllBoards: " << (ret ? "OK" : "FAILED") << std::endl;
 
        // Detailed status bits
        std::cout << "Status bits:" << std::endl;
        std::cout << "  Valid Read     = " << bool(boardStatus & 0x1) << std::endl;
        std::cout << "  Power Enable   = " << bool(boardStatus & 0x2) << std::endl;
        std::cout << "  Safety Relay   = " << bool(boardStatus & 0x4) << std::endl;
        std::cout << "  Watchdog       = " << bool(boardStatus & 0x8) << std::endl;
        std::cout << "  Board Power    = " << bool(boardStatus & 0x10) << std::endl;
        std::cout << "  Power Fault    = " << bool(boardStatus & 0x20) << std::endl;
        std::cout << "  Protocol Ver   = " << ((boardStatus >> 6) & 0x3) << std::endl;
        std::cout << "  Firmware Ver   = " << ((boardStatus >> 8) & 0xF) << std::endl;
        std::cout << "  Board ID       = " << ((boardStatus >> 16) & 0xFF) << "\n" << std::endl;
 
        // Print encoder values for each actuator.
        for (size_t i = 0; i < actuators.size(); i++) {
            unsigned int motorIdx = actuators[i].index - 1;
            int32_t rawVal = BoardList[0]->GetEncoderPosition(motorIdx);
            uint32_t midRange = BoardList[0]->GetEncoderMidRange();
            int32_t centered = rawVal - 0x800000;
 
            std::cout << "Actuator " << actuators[i].index 
                      << " | Raw: 0x" << std::hex << std::setw(6) << std::setfill('0') << rawVal
                      << " (" << std::dec << std::setw(8) << rawVal << ")"
                      << " | Centered: " << std::setw(8) << centered
                      << " | MidRange: 0x" << std::hex << midRange << std::dec << std::endl;
 
            double posSI = convertEncoderPosition(rawVal, actuators[i]);
            if (actuators[i].isPrismatic) {
                double posMM = posSI * 1000.0;
                std::cout << "         Position: " << std::fixed << std::setprecision(3)
                          << posSI << " m (" << posMM << " mm)";
            } else {
                double posDeg = posSI / deg2rad;
                std::cout << "         Position: " << std::fixed << std::setprecision(3)
                          << posSI << " rad (" << posDeg << " deg)";
            }
            
            if (actuators[i].hasLimits) {
                std::cout << " | Limits: [" 
                          << std::fixed << std::setprecision(3) << actuators[i].lowerLimit << ", "
                          << actuators[i].upperLimit << "]";
                // 10% margin (10% of the full limit range)
                double margin = (actuators[i].upperLimit - actuators[i].lowerLimit) * 0.1;
                if (posSI <= (actuators[i].lowerLimit + margin))
                    std::cout << " <-- Approaching Lower Limit";
                else if (posSI >= (actuators[i].upperLimit - margin))
                    std::cout << " <-- Approaching Upper Limit";
            }
            std::cout << std::endl << std::endl;
        }
 
        lastTime = currentTime;
 
        std::cout << "\nPress Enter to update, 'q' to quit: ";
        std::getline(std::cin, input);
        if (input == "q" || input == "Q")
            break;
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