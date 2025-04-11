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

// Structure for actuator parameters
struct ActuatorParams {
    int index;
    double encoderScale;         // Scale from raw encoder bits to physical units (per bit)
    std::string jointType;       // "REVOLUTE" or "PRISMATIC"
    bool isPrismatic;            // true if prismatic
    bool hasLimits;              // true if soft limits are specified
    double lowerLimit;           // Soft limit in SI (radians for revolute, meters for prismatic)
    double upperLimit;           // Soft limit in SI
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
        actuators.push_back(actuator);
    }
    inConfig.close();
    return actuators;
}
 
// Converts a raw encoder count to SI units.
double convertEncoderPosition(int32_t encoderCount, const ActuatorParams &params) {
    double posBits = static_cast<double>(encoderCount - 0x800000);
    double posUnits = posBits * params.encoderScale;
    double conversionFactor = params.isPrismatic ? mm2m : deg2rad;
    return posUnits * conversionFactor;
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

	Port->SetProtocol(BasePort::PROTOCOL_SEQ_RW);
	if (!Port->SetProtocol(BasePort::PROTOCOL_SEQ_RW)) {
		std::cerr << "Protocol set failed." << std::endl;
		return -1;
	}
 
	// Check if the board is connected.
	// check GetNumOfNodes()
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
 
    std::string input;
    double lastTime = Amp1394_GetTime();
    while (true) {
        bool ret = Port->ReadAllBoards();
		Amp1394_Sleep(0.0005);
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
				if (posSI <= actuators[i].lowerLimit)
					std::cout << " <-- At Lower Limit";
				else if (posSI >= actuators[i].upperLimit)
					std::cout << " <-- At Upper Limit";
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