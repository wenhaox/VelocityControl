/*
 * zynq_full.cpp
 *
 * Reads actuator configuration from the main config file (which contains soft position limits)
 * and runs a PD control loop. Soft limits for revolute joints are provided in degrees and converted to radians,
 * and for prismatic joints soft limits are provided in millimeters (converted to meters).
 *
 * Usage:
 *   ./zynq_full <config.json>
 *
 * Example configuration file: sawRobotIO1394-PSM1-334809.xml.json
 */
// TODO: pd control loop settings wjeret to read?
// set encoder preload how ot do?
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
 const double mm2m   = 0.001;
 
 // Structure for actuator parameters
 struct ActuatorParams {
	 int index;
	 double encoderScale;         // Scale from raw encoder bits to physical units (per bit)
	 std::string jointType;       // "REVOLUTE" or "PRISMATIC"
	 bool isPrismatic;            // true if prismatic
	 bool hasLimits;              // true if soft limits are specified
	 double lowerLimit;           // Lower soft limit in SI units (radians for revolute, meters for prismatic)
	 double upperLimit;           // Upper soft limit in SI units
	 ActuatorParams() : index(0), encoderScale(0.0), jointType("UNKNOWN"),
						isPrismatic(false), hasLimits(false), lowerLimit(0.0), upperLimit(0.0) {}
 };
 
 // Reads the actuator configuration from the main JSON config file.
 // It extracts encoder calibration and soft limits from each actuator.
 // For revolute joints with limits in "deg", values are converted to radians.
 // For prismatic joints with limits in "mm", values are converted to meters.
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
	 // JSON structure: { "Robots": [ { "Actuators": [ ... ] } ] }
	 Json::Value actuatorConfigs = robot_json["Robots"][0]["Actuators"];
	 if (actuatorConfigs.empty()) {
		 std::cerr << "No actuators found in configuration file." << std::endl;
		 exit(EXIT_FAILURE);
	 }
	 for (unsigned int i = 0; i < actuatorConfigs.size(); i++) {
		 ActuatorParams actuator;
		 actuator.index = i + 1;  // 1-indexed
		 actuator.encoderScale = actuatorConfigs[i]["Encoder"]["BitsToPosition"]["Scale"].asDouble();
		 actuator.jointType = actuatorConfigs[i]["JointType"].asString();
		 actuator.isPrismatic = (actuator.jointType == "PRISMATIC");
		 // If soft limits exist in the Encoder block, read and convert them.
		 if (actuatorConfigs[i]["Encoder"].isMember("PositionLimitsSoft")) {
			 Json::Value limitsSoft = actuatorConfigs[i]["Encoder"]["PositionLimitsSoft"];
			 actuator.hasLimits = true;
			 double lower = limitsSoft["Lower"].asDouble();
			 double upper = limitsSoft["Upper"].asDouble();
			 std::string unit = limitsSoft["Unit"].asString();
			 if (!actuator.isPrismatic) {
				 // Revolute joints: convert "deg" to radians.
				 if (unit == "deg") {
					 actuator.lowerLimit = lower * deg2rad;
					 actuator.upperLimit = upper * deg2rad;
				 } else {
					 actuator.lowerLimit = lower;
					 actuator.upperLimit = upper;
				 }
			 } else {
				 // Prismatic joints: convert "mm" to meters.
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
 // It subtracts the preload (0x800000), multiplies by the encoder scale, then by a conversion factor.
 // (For prismatic joints the conversion factor is mm2m; for revolute joints deg2rad is assumed.)
 double convertEncoderPosition(int32_t encoderCount, const ActuatorParams &params) {
	 double posBits = static_cast<double>(encoderCount - 0x800000);
	 double posUnits = posBits * params.encoderScale;
	 double conversionFactor = params.isPrismatic ? mm2m : deg2rad;
	 return posUnits * conversionFactor;
 }
 
 // Prints a summary of the actuator configurations to std::cout.
 void printActuatorInfo(const std::vector<ActuatorParams> &actuators) {
	 std::cout << "Actuator Config Summary:" << std::endl;
	 std::cout << std::setw(5)  << "Idx"
			   << std::setw(15) << "JointType"
			   << std::setw(15) << "EncScale"
			   << std::setw(10) << "Limits"
			   << std::setw(20) << "Lower (SI)"
			   << std::setw(20) << "Upper (SI)"
			   << std::endl;
	 for (const auto &actuator : actuators) {
		 std::cout << std::setw(5)  << actuator.index
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
	 // Expect one argument: the main configuration JSON file.
	 if (argc < 2) {
		 std::cerr << "Usage: " << argv[0] << " <config.json>" << std::endl;
		 return -1;
	 }
	 std::string configFile = argv[1];
	 std::vector<ActuatorParams> actuators = readActuatorConfigs(configFile);
	 
	 // For debugging, print out actuator configurations
	 printActuatorInfo(actuators);
	 
	 // Hardware initialization.
	 unsigned int numDisp = 1; // Assume one board/display.
	 std::vector<AmpIO*> BoardList;
	 BoardList.push_back(new AmpIO(0));
	 
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
	 
	 if (!Port->SetProtocol(BasePort::PROTOCOL_SEQ_RW)) {
		 std::cerr << "Protocol set failed." << std::endl;
	 }
	 
	 const unsigned int MAX_AXES = 11;
	 uint32_t MotorCurrents[MAX_AXES];
	 for (unsigned int i = 0; i < MAX_AXES; i++)
		 MotorCurrents[i] = 0x8000;
	 
	 // Power up boards.
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
		 double dt = currentTime - lastTime;
		 if (dt <= 0.0) dt = 0.001;
		 lastTime = currentTime;
		 
		 // Update actuator feedback and apply PD control.
		 for (size_t i = 0; i < actuators.size(); i++) {
			 unsigned int motorIdx = actuators[i].index - 1;
			 int32_t rawPos = BoardList[0]->GetEncoderPosition(motorIdx);
			 uint32_t velPred = BoardList[0]->GetEncoderVelocityPredicted(motorIdx);
			 double posSI = convertEncoderPosition(rawPos, actuators[i]);
			 double conversionFactor = actuators[i].isPrismatic ? mm2m : deg2rad;
			 double velSI = static_cast<double>(velPred) * conversionFactor;
			 
			 // PD control computation
			 double desiredVel = 0.0; // Constant desired velocity; adjust as needed.
			 double error = desiredVel - velSI;
			 double derivative = (error - prevError[i]) / dt;
			 prevError[i] = error;
			 double controlEffort = Kp * error + Kd * derivative;
			 
			 // Enforce safety zone near soft limits.
			 bool safetyActive = false;
			 if (actuators[i].hasLimits) {
				 double range = actuators[i].upperLimit - actuators[i].lowerLimit;
				 double stopZone = 0.05 * range; // 5% safety zone.
				 if (posSI <= (actuators[i].lowerLimit + stopZone) ||
					 posSI >= (actuators[i].upperLimit - stopZone)) {
					 controlEffort = 0.0;
					 safetyActive = true;
					 // Optionally display a safety active message (here printed on row 4, but will be overwritten below).
					 console.Print(4, 5, "Actuator %d safety zone active", actuators[i].index);
				 }
			 }
			 
			 uint32_t commandedCurrent = 0x8000 + static_cast<uint32_t>(controlEffort);
			 BoardList[0]->SetMotorCurrent(motorIdx, commandedCurrent);
			 
			 // Prepare feedback string based on joint type.
			 char msg[256];
			 if (actuators[i].isPrismatic) {
				 double posMM = posSI * 1000.0;
				 double velMM = velSI * 1000.0;
				 snprintf(msg, sizeof(msg),
						  "Actuator %d | Pos: %.3f m (%.1f mm) | Vel: %.3f m/s (%.1f mm/s)%s",
						  actuators[i].index, posSI, posMM, velSI, velMM,
						  safetyActive ? " | ** Safety zone active **" : "");
			 } else {
				 double posDeg = posSI / deg2rad;
				 double velDeg = velSI / deg2rad;
				 snprintf(msg, sizeof(msg),
						  "Actuator %d | Pos: %.3f rad (%.1f deg) | Vel: %.3f rad/s (%.1f deg/s)%s",
						  actuators[i].index, posSI, posDeg, velSI, velDeg,
						  safetyActive ? " | ** Safety zone active **" : "");
			 }
			 // Print this actuator’s feedback on a fixed row (row 4 + actuator index).
			 console.Print(4 + i, 5, msg);
		 }
		 
		 Port->WriteAllBoards();
		 console.Refresh();
		 if (Port->GetPortType() != BasePort::PORT_ZYNQ_EMIO)
			 Amp1394_Sleep(0.0005);
	 }
	 
	 // Shutdown sequence.
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