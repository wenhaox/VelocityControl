/*
 * zynq_full.cpp
 * Reads actuator configuration from the main config file and overrides joint limits
 * using values read from a separate limits file.
 */

 #include <stdlib.h>
 #include <iostream>
 #include <fstream>
 #include <cmath>
 #include <string>
 #include <vector>
 #include <iomanip>
 #include "jsoncpp/include/json/json.h"
 
 #include <Amp1394/AmpIORevision.h>
 #include "PortFactory.h"
 #include "AmpIO.h"
 #include "Amp1394Time.h"
 #include "Amp1394Console.h"
 #include "EthBasePort.h"
 
 // Conversion factors
 const double deg2rad = M_PI / 180.0;
 const double mm2m = 0.001;
 
 // Structure for actuator parameters
 struct ActuatorParams {
	 int index;
	 double encoderScale;
	 std::string jointType;
	 bool isPrismatic;
	 bool hasLimits;
	 double lowerLimit;
	 double upperLimit;
	 ActuatorParams() : index(0), encoderScale(0.0), jointType("UNKNOWN"),
						isPrismatic(false), hasLimits(false), lowerLimit(0.0), upperLimit(0.0) {}
 };
 
 // Read general actuator configuration from main config file (no limits here)
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
	 Json::Value actuatorConfigs = robot_json["Robots"][0]["Actuators"];
	 if (actuatorConfigs.empty()) {
		 std::cerr << "No actuators found in configuration file." << std::endl;
		 exit(EXIT_FAILURE);
	 }
	 for (unsigned int i = 0; i < actuatorConfigs.size(); i++) {
		 ActuatorParams actuator;
		 actuator.index = i + 1; // Use 1-indexed IDs
		 actuator.encoderScale = actuatorConfigs[i]["Encoder"]["BitsToPosition"]["Scale"].asDouble();
		 actuator.jointType = actuatorConfigs[i]["JointType"].asString();
		 actuator.isPrismatic = (actuator.jointType == "PRISMATIC");
		 // Limits are not provided in this file
		 actuators.push_back(actuator);
	 }
	 inConfig.close();
	 return actuators;
 }
  
 // Read joint limits from limits file (using "qmin" and "qmax")
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
		 double conversionFactor = isPrismatic ? mm2m : 1.0;  // Revolute joints already in SI
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
  
 // Print actuator configuration summary
 void printActuatorInfo(const std::vector<ActuatorParams> &actuators) {
	 std::cout << "Actuator Config Summary:" << std::endl;
	 std::cout << "-------------------------------------------------" << std::endl;
	 std::cout << std::setw(5)  << "Idx" 
			   << std::setw(15) << "Joint Type" 
			   << std::setw(15) << "EncoderScale" 
			   << std::setw(15) << "Has Limits" 
			   << std::setw(15) << "Lower Limit" 
			   << std::setw(15) << "Upper Limit" 
			   << std::endl;
	 std::cout << "-------------------------------------------------" << std::endl;
	 for (const auto &actuator : actuators) {
		 std::cout << std::setw(5)  << actuator.index
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
	 std::cout << "-------------------------------------------------" << std::endl;
 }
  
 int main(int argc, char** argv) {
	 // Expect two arguments: main config and limits config
	 if (argc < 3) {
		 std::cerr << "Usage: " << argv[0] << " <main_config.json> <limits_config.json>" << std::endl;
		 return -1;
	 }
	 std::string mainConfigFile = argv[1];
	 std::string limitsConfigFile = argv[2];
 
	 // Read main config for actuators (without limits)
	 std::vector<ActuatorParams> actuators = readActuatorConfigs(mainConfigFile);
	 
	 // Override actuator limits using values from limits file
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
 
	 // Setup port/boards (similar to qladisp)
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
		 std::cerr << "Failed to set protocol." << std::endl;
	 
	 // Initialize motor currents at mid-range (0x8000)
	 const unsigned int MAX_AXES = 11;
	 uint32_t MotorCurrents[MAX_AXES];
	 for (unsigned int i = 0; i < MAX_AXES; i++)
		 MotorCurrents[i] = 0x8000;
	 
	 // Power up boards
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
	 console.Print(1, 5, "Zynq PD Control Test");
	 console.Print(2, 5, "Press ESC to quit");
	 console.Refresh();
	 
	 // PD Control Loop
	 std::vector<double> prevError(actuators.size(), 0.0);
	 const double Kp = 100.0, Kd = 10.0;
	 double startTime = Amp1394_GetTime();
	 double lastTime = startTime;
	 
	 const int ESC_CHAR = 0x1b;
	 int c;
	 while ((c = console.GetChar()) != ESC_CHAR) {
		 Port->ReadAllBoards();
		 double currentTime = Amp1394_GetTime();
		 double dt = (currentTime - lastTime);
		 if (dt <= 0.0) dt = 0.001;
		 lastTime = currentTime;
		 
		 for (size_t i = 0; i < actuators.size(); i++) {
			 unsigned int motorIdx = actuators[i].index - 1;
			 int32_t rawPos = BoardList[0]->GetEncoderPosition(motorIdx);
			 uint32_t velPred = BoardList[0]->GetEncoderVelocityPredicted(motorIdx);
			 double posSI = convertEncoderPosition(rawPos, actuators[i]);
			 double conversionFactor = actuators[i].isPrismatic ? mm2m : deg2rad;
			 double velSI = static_cast<double>(velPred) * conversionFactor;
			 double desiredVel = 0.0;
			 double error = desiredVel - velSI;
			 double derivative = (error - prevError[i]) / dt;
			 prevError[i] = error;
			 double controlEffort = Kp * error + Kd * derivative;
			 
			 // Enforce safety zone if near joint limits
			 if (actuators[i].hasLimits) {
				 double range = actuators[i].upperLimit - actuators[i].lowerLimit;
				 double stopZone = 0.05 * range; // 5% of range
				 if (posSI <= (actuators[i].lowerLimit + stopZone) ||
					 posSI >= (actuators[i].upperLimit - stopZone)) {
					 controlEffort = 0.0;
					 console.Print(4, 5, "Actuator %d safety zone active", actuators[i].index);
				 }
			 }
			 
			 uint32_t commandedCurrent = 0x8000 + static_cast<uint32_t>(controlEffort);
			 BoardList[0]->SetMotorCurrent(motorIdx, commandedCurrent);
		 }
		 
		 Port->WriteAllBoards();
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