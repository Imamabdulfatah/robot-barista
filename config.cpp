#include "stdafx.h"
#include "config.h"
#include <windows.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>

RobotAppConfig g_appConfig;

namespace {

std::string trim(const std::string& value)
{
	const auto start = value.find_first_not_of(" \t\r\n");
	if (start == std::string::npos) {
		return "";
	}
	const auto end = value.find_last_not_of(" \t\r\n");
	return value.substr(start, end - start + 1);
}

std::string toLower(std::string value)
{
	std::transform(value.begin(), value.end(), value.begin(),
		[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	return value;
}

void applyDefaults(RobotAppConfig& config)
{
	config.robotPort = 8899;
	config.comHand = 3;
	config.arduinoPort = "COM6";
	config.arduinoBaud = 9600;
	config.tcpBridgePort = 8765;
	config.robotAddr.clear();
	config.apiToken.clear();
}

bool readEnvString(const char* name, std::string& target)
{
	char buffer[512] = {};
	const DWORD length = GetEnvironmentVariableA(name, buffer, sizeof(buffer));
	if (length == 0 || length >= sizeof(buffer)) {
		return false;
	}
	target = trim(std::string(buffer, length));
	return !target.empty();
}

bool readEnvInt(const char* name, int& target)
{
	char buffer[64] = {};
	const DWORD length = GetEnvironmentVariableA(name, buffer, sizeof(buffer));
	if (length == 0 || length >= sizeof(buffer)) {
		return false;
	}
	target = atoi(buffer);
	return true;
}

bool readEnvULong(const char* name, unsigned long& target)
{
	char buffer[64] = {};
	const DWORD length = GetEnvironmentVariableA(name, buffer, sizeof(buffer));
	if (length == 0 || length >= sizeof(buffer)) {
		return false;
	}
	target = strtoul(buffer, NULL, 10);
	return true;
}

void applyEnvironmentOverrides(RobotAppConfig& config)
{
	readEnvString("ROBOT_ADDR", config.robotAddr);
	readEnvInt("ROBOT_PORT", config.robotPort);
	readEnvInt("COM_HAND", config.comHand);
	readEnvString("ARDUINO_PORT", config.arduinoPort);
	readEnvULong("ARDUINO_BAUD", config.arduinoBaud);
	readEnvInt("TCP_BRIDGE_PORT", config.tcpBridgePort);
	readEnvString("ROBOT_API_TOKEN", config.apiToken);
}

bool applyIniValue(RobotAppConfig& config, const std::string& keyRaw, const std::string& valueRaw)
{
	const std::string key = toLower(keyRaw);
	const std::string value = trim(valueRaw);

	if (key == "robot_addr" || key == "robot_ip") {
		config.robotAddr = value;
		return true;
	}
	if (key == "robot_port") {
		config.robotPort = atoi(value.c_str());
		return true;
	}
	if (key == "com_hand") {
		config.comHand = atoi(value.c_str());
		return true;
	}
	if (key == "arduino_port") {
		config.arduinoPort = value;
		return true;
	}
	if (key == "arduino_baud") {
		config.arduinoBaud = strtoul(value.c_str(), NULL, 10);
		return true;
	}
	if (key == "tcp_bridge_port") {
		config.tcpBridgePort = atoi(value.c_str());
		return true;
	}
	if (key == "api_token" || key == "robot_api_token") {
		config.apiToken = value;
		return true;
	}

	return false;
}

} // namespace

bool loadRobotAppConfig(const char* configPath)
{
	applyDefaults(g_appConfig);

	std::ifstream file(configPath);
	if (file.is_open()) {
		std::string line;
		while (std::getline(file, line)) {
			line = trim(line);
			if (line.empty() || line[0] == '#' || line[0] == ';') {
				continue;
			}

			const size_t separator = line.find('=');
			if (separator == std::string::npos) {
				continue;
			}

			const std::string key = trim(line.substr(0, separator));
			const std::string value = trim(line.substr(separator + 1));
			applyIniValue(g_appConfig, key, value);
		}
		printf("Config dimuat dari %s\n", configPath);
	}
	else {
		printf("File %s tidak ditemukan. Gunakan env vars atau salin robot.config.example.ini\n", configPath);
	}

	applyEnvironmentOverrides(g_appConfig);

	if (g_appConfig.robotAddr.empty()) {
		printf("ERROR: ROBOT_ADDR belum diatur (robot.config.ini atau env ROBOT_ADDR)\n");
		return false;
	}

	if (g_appConfig.apiToken.empty()) {
		printf("PERINGATAN: ROBOT_API_TOKEN kosong. TCP bridge tanpa autentikasi (hanya untuk dev lokal).\n");
	}

	return true;
}

bool isTcpMessageAuthorized(const std::string& message, std::string& payloadOut)
{
	payloadOut = trim(message);

	if (g_appConfig.apiToken.empty()) {
		return true;
	}

	const size_t separator = payloadOut.find('|');
	if (separator == std::string::npos) {
		return false;
	}

	const std::string token = trim(payloadOut.substr(0, separator));
	payloadOut = trim(payloadOut.substr(separator + 1));

	return token == g_appConfig.apiToken;
}
