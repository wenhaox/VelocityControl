#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>
#include <iomanip> // for std::setprecision
#include "jsoncpp/include/json/json.h" // Requires jsoncpp
//TODO: Joint limit(PID config file for joint limit), Create safety zone near limits (10% of range or fixed value), need an actual captture file for the machine, will the json file work if we got it from the machine work 
// test joint limit wiht hand first and see for a printout
// get the feedback from the board, do the control computation, and then write the motor currents.  The first is done by calling ReadAllBoards in the port class (in your case, ZynqEmioPort).  Once you get the feedback, you would call AmpIO::GetEncoderVelocityPredicted to get the velocity (in counts/sec).  For writing the motor current, you would call AmpIO::SetMotorCurrent to build the local buffer, and then ZynqEmioPort::WriteAllBoards to actually send it to the FPGA. 
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

// Test encoder position conversion for a specific motor
void testEncoderPosition(int motorIndex, int32_t encoderCount, const std::vector<ActuatorParams> &actuators) {
    // Check if motor index is valid
    if (motorIndex < 1 || motorIndex > static_cast<int>(actuators.size())) {
        std::cerr << "Invalid motor index: " << motorIndex << std::endl;
        std::cerr << "Valid range is 1 to " << actuators.size() << std::endl;
        return;
    }
    
    // Get actuator parameters for the specified motor
    const ActuatorParams &actuator = actuators[motorIndex - 1]; // Convert to 0-based index
    
    // Convert position
    double positionSI = convertEncoderPosition(encoderCount, actuator);
    
    // Check limits
    bool withinLimits = checkPositionLimits(positionSI, actuator);
    
    // Print results
    std::cout << "\nEncoder Position Conversion Test for Motor " << motorIndex << ":" << std::endl;
    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "Raw Encoder Count: " << encoderCount << std::endl;
    std::cout << "With 0x800000 offset: " << (encoderCount - 0x800000) << std::endl;
    std::cout << "Joint Type: " << actuator.jointType << std::endl;
    std::cout << "Encoder Scale: " << actuator.encoderScale << std::endl;
    std::cout << "Conversion Factor: " << (actuator.isPrismatic ? mm2m : deg2rad) << std::endl;
    std::cout << "Converted Position: " << positionSI << " " 
              << (actuator.isPrismatic ? "meters" : "radians") << std::endl;
    
    if (actuator.hasLimits) {
        std::cout << "Joint Limits: [" << actuator.lowerLimit << ", " << actuator.upperLimit << "] " 
                  << (actuator.isPrismatic ? "meters" : "radians") << std::endl;
        std::cout << "Within Limits: " << (withinLimits ? "Yes" : "No") << std::endl;
    } else {
        std::cout << "Joint Limits: Not defined" << std::endl;
    }
    std::cout << "--------------------------------------------------" << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config.json>" << std::endl;
        std::cerr << "  - config.json: Path to the robot configuration file" << std::endl;
        return -1;
    }

    std::string configFile = argv[1];
    
    // Read actuator configurations
    std::vector<ActuatorParams> actuators = readActuatorConfigs(configFile);
    
    // Print summary of all actuators
    printActuatorInfo(actuators);
    
    // Define test data - encoder counts and expected values for motors 1-7
    std::vector<int32_t> encoderCounts = {8391286, 8394466, 7459811, 8388610, 8388608, 8388595, 8388558};
    std::vector<double> expectedValues = {0.002464801315906465, -0.005285917326719919, 0.12083540215580316, 
                                        -0.00022743738895170438, -0.0, -0.0014783430281860785, -0.00568593472379261};
    
    if (actuators.size() < 7) {
        std::cerr << "Warning: Configuration has fewer than 7 actuators." << std::endl;
        std::cerr << "Only processing " << actuators.size() << " actuators." << std::endl;
    }
    
    std::cout << "\nTesting specific encoder values for all motors:" << std::endl;
    for (size_t i = 0; i < std::min(encoderCounts.size(), actuators.size()); ++i) {
        int motorIndex = i + 1;
        int32_t encoderCount = encoderCounts[i];
        
        // Get actuator parameters
        const ActuatorParams &actuator = actuators[i];
        
        // Convert position
        double positionSI = convertEncoderPosition(encoderCount, actuator);
        
        // Check against expected value
        double expectedPos = expectedValues[i];
        double difference = fabs(positionSI - expectedPos);
        bool matches = difference < 1e-8;
        
        // Check limits
        bool withinLimits = checkPositionLimits(positionSI, actuator);
        
        // Print results
        std::cout << "\nEncoder Position Test for Motor " << motorIndex << ":" << std::endl;
        std::cout << "--------------------------------------------------" << std::endl;
        std::cout << "Raw Encoder Count: " << encoderCount << std::endl;
        std::cout << "With 0x800000 offset: " << (encoderCount - 0x800000) << std::endl;
        std::cout << "Joint Type: " << actuator.jointType << std::endl;
        std::cout << "Encoder Scale: " << actuator.encoderScale << std::endl;
        std::cout << "Conversion Factor: " << (actuator.isPrismatic ? mm2m : deg2rad) << std::endl;
        
        std::cout << "Converted Position: " << std::setprecision(15) << positionSI << " " 
                << (actuator.isPrismatic ? "meters" : "radians") << std::endl;
        std::cout << "Expected Position: " << std::setprecision(15) << expectedPos << " "
                << (actuator.isPrismatic ? "meters" : "radians") << std::endl;
        std::cout << "Difference: " << difference << (matches ? " (MATCHES)" : " (DOES NOT MATCH)") << std::endl;
        
        if (actuator.hasLimits) {
            std::cout << "Joint Limits: [" << actuator.lowerLimit << ", " << actuator.upperLimit << "] " 
                    << (actuator.isPrismatic ? "meters" : "radians") << std::endl;
            std::cout << "Within Limits: " << (withinLimits ? "Yes" : "No") << std::endl;
        } else {
            std::cout << "Joint Limits: Not defined" << std::endl;
        }
        std::cout << "--------------------------------------------------" << std::endl;
    }
    
    return 0;
}