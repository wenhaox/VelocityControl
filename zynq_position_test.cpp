/*
 * zynq_position_test.cpp
 * Reads board sensor feedback and prints joint status.
 * Usage: ./zynq_full <main_config.json> <limits_config.json>
 */

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
 
 #include <Amp1394/AmpIORevision.h>
 #include "PortFactory.h"
 #include "AmpIO.h"
 #include "Amp1394Time.h"
 #include "Amp1394Console.h"
 #include "EthBasePort.h"
 
 // Unit conversion factors
 const double deg2rad = M_PI / 180.0;
 const double mm2m = 0.001;
 
 // Actuator parameters structure
 struct ActuatorParams {
	 int index;
	 double encoderScale;
	 std::string jointType;
	 bool isPrismatic;
	 bool hasLimits;
	 double lowerLimit;
	 double upperLimit;
	 ActuatorParams() : index(0), encoderScale(0.0), jointType("UNKNOWN"),
						isPrismatic(false), hasLimits(false),
						lowerLimit(0.0), upperLimit(0.0) {}
 };
 
 // Read actuator config from main JSON file (no limits here)
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
		 std::cerr << "Failed to parse JSON configuration." << std::endl;
		 std::cerr << reader.getFormattedErrorMessages() << std::endl;
		 exit(EXIT_FAILURE);
	 }
	 // Expect the main config to have { "Robots": [ { "Actuators": [ { ... } ] } ] }
	 Json::Value actuatorConfigs = robot_json["Robots"][0]["Actuators"];
	 if (actuatorConfigs.empty()) {
		 std::cerr << "No actuators found in configuration file." << std::endl;
		 exit(EXIT_FAILURE);
	 }
	 for (unsigned int i = 0; i < actuatorConfigs.size(); i++) {
		 ActuatorParams actuator;
		 actuator.index = i + 1; // 1-indexed
		 // Extract encoder parameters
		 actuator.encoderScale = actuatorConfigs[i]["Encoder"]["BitsToPosition"]["Scale"].asDouble();
		 // Get joint type and determine if prismatic
		 actuator.jointType = actuatorConfigs[i]["JointType"].asString();
		 actuator.isPrismatic = (actuator.jointType == "PRISMATIC");
		 // (No limits read from this file)
		 actuators.push_back(actuator);
	 }
	 inConfig.close();
	 return actuators;
 }
 
 // Read joint limits from psm-si.json (using "qmin" and "qmax")
 std::map<int, std::pair<double,double>> readActuatorLimits(const std::string &limitsFile) {
	 std::map<int, std::pair<double,double>> limits;
	 std::ifstream inLimits(limitsFile);
	 if (!inLimits.is_open()) {
		 std::cerr << "Failed to open limits file: " << limitsFile << std::endl;
		 exit(EXIT_FAILURE);
	 }
	 Json::Value json;
	 Json::Reader reader;
	 if (!reader.parse(inLimits, json)) {
		 std::cerr << "Limits JSON parse error:" << std::endl;
		 std::cerr << reader.getFormattedErrorMessages() << std::endl;
		 exit(EXIT_FAILURE);
	 }
	 Json::Value joints = json["DH"]["joints"];
	 if (joints.empty()) {
		 std::cerr << "No joints found in limits file." << std::endl;
		 exit(EXIT_FAILURE);
	 }
	 for (unsigned int i = 0; i < joints.size(); i++) {
		 int label = std::stoi(joints[i]["label"].asString());
		 double qmin = joints[i]["qmin"].asDouble();
		 double qmax = joints[i]["qmax"].asDouble();
		 std::string type = joints[i]["type"].asString();
		 bool isPrismatic = (type == "PRISMATIC" || type == "prismatic");
		 double conversionFactor = isPrismatic ? mm2m : 1.0; // Revolute joints are assumed in SI.
		 limits[label] = std::make_pair(qmin * conversionFactor, qmax * conversionFactor);
	 }
	 inLimits.close();
	 return limits;
 }
 
 // Convert raw encoder count to SI units
 double convertEncoderPosition(int32_t encoderCount, const ActuatorParams &params) {
	 double posBits = static_cast<double>(encoderCount - 0x800000);
	 double posUnits = posBits * params.encoderScale;
	 double conversionFactor = params.isPrismatic ? mm2m : deg2rad;
	 return posUnits * conversionFactor;
 }
 
 // Print configuration summary.
 void printActuatorInfo(const std::vector<ActuatorParams> &actuators) {
	 std::cout << "Actuator Config Summary:" << std::endl;
	 std::cout << "Idx   Joint Type     EncoderScale    HasLimits    LowerLimit    UpperLimit" << std::endl;
	 for (const auto &actuator : actuators) {
		 std::cout << std::setw(5) << actuator.index
				   << std::setw(15) << actuator.jointType
				   << std::setw(15) << std::fixed << std::setprecision(8) << actuator.encoderScale
				   << std::setw(15) << (actuator.hasLimits ? "Yes" : "No");
		 if (actuator.hasLimits)
			 std::cout << std::setw(15) << std::fixed << std::setprecision(6) << actuator.lowerLimit
					   << std::setw(15) << std::fixed << std::setprecision(6) << actuator.upperLimit;
		 else
			 std::cout << std::setw(15) << "N/A" << std::setw(15) << "N/A";
		 std::cout << std::endl;
	 }
	 std::cout << std::string(70, '-') << std::endl;
 }
  
 int main(int argc, char** argv) {
	 if (argc < 3) {
		 std::cerr << "Usage: " << argv[0] << " <main_config.json> <limits_config.json>" << std::endl;
		 return -1;
	 }
	 std::string mainConfigFile = argv[1];
	 std::string limitsConfigFile = argv[2];
 
	 // Read general actuator config from the main config file.
	 std::vector<ActuatorParams> actuators = readActuatorConfigs(mainConfigFile);
	 
	 // Read limits from psm-si.json and override limits in actuators.
	 std::map<int, std::pair<double,double>> limits = readActuatorLimits(limitsConfigFile);
	 for (auto &actuator : actuators) {
		 auto it = limits.find(actuator.index);
		 if (it != limits.end()) {
			 actuator.hasLimits = true;
			 actuator.lowerLimit = it->second.first;
			 actuator.upperLimit = it->second.second;
		 }
	 }
	 
	 printActuatorInfo(actuators);
  
	 // Setup port and board (similar to qladisp)
	 unsigned int numDisp = 0;
	 std::vector<AmpIO*> BoardList;
	 BoardList.push_back(new AmpIO(0));
	 numDisp = 1;
  
	 std::string portDescription = BasePort::DefaultPort();
	 std::stringstream debugStream;
	 BasePort::AddHardwareVersionStringList("");
	 BasePort *Port = PortFactory(portDescription.c_str(), debugStream);
	 if (!Port || !Port->IsOK()) {
		 std::cerr << "Port init failed" << std::endl;
		 return -1;
	 }
	 for (size_t i = 0; i < BoardList.size(); i++)
		 Port->AddBoard(BoardList[i]);
  
	 if (!Port->SetProtocol(BasePort::PROTOCOL_SEQ_RW))
		 std::cerr << "Protocol set failed." << std::endl;
  
	 const unsigned int MAX_AXES = 11;
	 uint32_t MotorCurrents[MAX_AXES];
	 for (unsigned int i = 0; i < MAX_AXES; i++)
		 MotorCurrents[i] = 0x8000;
  
	 for (unsigned int j = 0; j < numDisp; j++) {
		 BoardList[j]->WriteSafetyRelay(true);
		 BoardList[j]->WritePowerEnable(true);
		 BoardList[j]->WriteAmpEnable(0x0f, 0x0f);
	 }
  
	 // Setup console (qladisp style)
	 Amp1394Console console;
	 console.Init();
	 if (!console.IsOK()) {
		 std::cerr << "Console init failed" << std::endl;
		 return -1;
	 }
	 console.Print(1, 5, "Zynq Joint Limit Test");
	 console.Print(2, 5, "Press ESC to quit");
	 console.Refresh();
  
	 // Main loop: read sensor feedback and print joint status.
	 const int ESC_CHAR = 0x1b;
	 int c;
	 while ((c = console.GetChar()) != ESC_CHAR) {
		 Port->ReadAllBoards();
		 for (size_t i = 0; i < actuators.size(); i++) {
			 unsigned int motorIdx = actuators[i].index - 1;
			 int32_t rawPos = BoardList[0]->GetEncoderPosition(motorIdx);
			 uint32_t velPred = BoardList[0]->GetEncoderVelocityPredicted(motorIdx);
			 double posSI = convertEncoderPosition(rawPos, actuators[i]);
			 double conversionFactor = actuators[i].isPrismatic ? mm2m : deg2rad;
			 double velSI = static_cast<double>(velPred) * conversionFactor;
			 bool safetyActive = false;
			 if (actuators[i].hasLimits) {
				 double range = actuators[i].upperLimit - actuators[i].lowerLimit;
				 double stopZone = 0.05 * range;
				 if (posSI <= (actuators[i].lowerLimit + stopZone) ||
					 posSI >= (actuators[i].upperLimit - stopZone))
					 safetyActive = true;
			 }
			 std::cout << "Act " << actuators[i].index 
					   << " | Pos: " << posSI 
					   << " | Vel: " << velSI;
			 if (safetyActive)
				 std::cout << " | ** Safety zone active **";
			 std::cout << std::endl;
		 }
		 std::cout << "---------------------------------------------" << std::endl;
		 console.Refresh();
		 if (Port->GetPortType() != BasePort::PORT_ZYNQ_EMIO)
			 Amp1394_Sleep(0.0005);
	 }
  
	 // Shutdown sequence
	 for (unsigned int j = 0; j < numDisp; j++) {
		 BoardList[j]->WritePowerEnable(false);
		 BoardList[j]->WriteAmpEnable(0x0f, 0x00);
		 BoardList[j]->WriteSafetyRelay(false);
	 }
	 console.End();
	 for (size_t j = 0; j < BoardList.size(); j++) {
		 Port->RemoveBoard(BoardList[j]->GetBoardId());
		 delete BoardList[j];
	 }
	 delete Port;
	 return 0;
 }