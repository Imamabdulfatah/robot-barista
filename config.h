#pragma once

#include <string>

struct RobotAppConfig {
	std::string robotAddr;
	int robotPort;
	int comHand;
	std::string arduinoPort;
	unsigned long arduinoBaud;
	int tcpBridgePort;
	std::string apiToken;
};

extern RobotAppConfig g_appConfig;

bool loadRobotAppConfig(const char* configPath = "robot.config.ini");

bool isTcpMessageAuthorized(const std::string& message, std::string& payloadOut);
