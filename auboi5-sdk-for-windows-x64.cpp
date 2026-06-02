#include "stdafx.h"
#include "rsdef.h"
#include <string>
#include "config.h"
#include "example.h"
#include <iostream>
#include <serial/serial.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <queue>
#include <mutex>

#pragma comment(lib, "Ws2_32.lib")

extern RSHD g_rshd;

namespace {

constexpr int SPEED_FAST = 10;
constexpr int SPEED_SLOW = 4;

std::mutex g_commandQueueMutex;
std::queue<std::string> g_commandQueue;
volatile bool g_appRunning = true;

constexpr double JOINT_VELC_SCALE = 178.2 / 100.0;
constexpr double JOINT_ACC_SCALE = 1782.0 / 100.0;
constexpr int JOINT_ACC_FACTOR = 10;

struct JointPose {
	const char* name;
	double j1, j2, j3, j4, j5, j6;
};

const JointPose kRobotPoses[] = {
	{ "home",           -13.0230, -65.1843, 105.4089,  84.1244,  89.2635,  13.2763 },
	{ "siapAmbilGelas", -13.8790, -54.4310, 110.3245,  75.4855,  89.6809,  -4.3400 },
	{ "AmbilGelas",     -23.2090, -93.6284,  75.9491,  80.3501,  89.8039, -13.6698 },
	{ "siapSeduhKopi",  -14.8020, -68.1999,  78.2886,  57.2241,  89.6929,  -5.2639 },
	{ "SeduhKopi",      -14.8030, -75.9731,  73.5719,  60.2807,  89.6929,  -5.2641 },
	{ "sajikanKopiA",   -14.8020, -52.9164,  83.1394,  52.2228,  89.7805,  -5.2641 },
	{ "sajikanKopiA2",  -12.4942, -51.9417, 106.4032, 117.1420,  87.8412,  -2.7983 },
	{ "sajikanKopiB",   -11.2368, -40.7410,  97.3220,  54.7000,  86.7334,  -1.7180 },
	{ "sajikanKopiB2",  -10.7960, -39.8221,  91.9789,  75.3619,  86.6823,  -1.2794 },
	{ "sajikanKopiC",   -11.8731, -21.8830,  95.4520,  30.9771,  87.4049,  -2.3552 },
	{ "sajikanKopiC2",  -12.6465, -18.3861,  11.6990,  75.0404,  87.9515,  -2.9096 },
};

void fillJointArray(double target[6], double value)
{
	for (int i = 0; i < 6; ++i) {
		target[i] = value;
	}
}

void pulseDigitalOutput(void (*setHigh)(RSHD), void (*setLow)(RSHD), DWORD holdMs)
{
	setHigh(g_rshd);
	Sleep(holdMs);
	setLow(g_rshd);
	Sleep(holdMs);
}

void enqueueRobotCommand(const std::string& command)
{
	std::lock_guard<std::mutex> lock(g_commandQueueMutex);
	g_commandQueue.push(command);
	printf("Perintah masuk antrian: %s\n", command.c_str());
}

bool dequeueRobotCommand(std::string& command)
{
	std::lock_guard<std::mutex> lock(g_commandQueueMutex);
	if (g_commandQueue.empty()) {
		return false;
	}
	command = g_commandQueue.front();
	g_commandQueue.pop();
	return true;
}

} // namespace

RSHD g_rshd = -1;

double User_jointMaxAcc[6] = { 0 };
double User_jointMaxVelc[6] = { 0 };
double User_jointAngle[6] = { 0 };
Pos User_Pos;
int Show_Status = 0;
int Joint_Delay = 0;

extern int State_Btn;

int State_Btn_Cur = 0;
int State_Btn_Pre = 0;
int State_Btn_Uesd = 0;
int State_Check_Flag = 0;

int cepat = SPEED_FAST;
int lambat = SPEED_SLOW;

static void gripGenggam()
{
	Com_Hands_AngleSet(100, 100, 100, 1000, 100, 0);
	Sleep(1000);
	Com_Hands_AngleSet(100, 100, 100, 100, 100, 0);
	Sleep(1000);
}

static void gripLepas()
{
	Com_Hands_AngleSet(1000, 1000, 1000, 1000, 1000, 0);
}

void pos(const std::string& posisi)
{
	for (const auto& pose : kRobotPoses) {
		if (posisi == pose.name) {
			JXB_move_GoToJoint(g_rshd, pose.j1, pose.j2, pose.j3, pose.j4, pose.j5, pose.j6);
			return;
		}
	}
}

void setKec(int kecepatan)
{
	printf("Kecepatan: %d \n", kecepatan);

	const double maxVelc = kecepatan * JOINT_VELC_SCALE;
	const double maxAcc = JOINT_ACC_FACTOR * JOINT_ACC_SCALE;

	fillJointArray(User_jointMaxVelc, maxVelc);
	fillJointArray(User_jointMaxAcc, maxAcc);
	JXB_move_SetPara(g_rshd);
}

void mesinKopiOn()
{
	pulseDigitalOutput(pin_D0_00_High, pin_D0_00_Low, 1000);
}

void suaraPra()
{
	pulseDigitalOutput(pin_D0_01_High, pin_D0_01_Low, 500);
}

void suaraAfter()
{
	pulseDigitalOutput(pin_D0_02_High, pin_D0_02_Low, 500);
}

void tungguMesin(int tunggu)
{
	printf("oke.. tunggu mesin");

	for (int i = 0; i <= tunggu; ++i) {
		Sleep(1000);
		printf("tunggu mesin %i \n", i);
	}

	printf("mesin kopi selesai");
}

void persiapan()
{
	setKec(cepat);
	suaraPra();
	pos("home");
	gripLepas();

	Sleep(1000);
	pos("siapAmbilGelas");
	Sleep(1000);

	if (pin_DI_07(g_rshd) == 1) {
		pos("ambilGelas");
		gripGenggam();
		Sleep(500);
		pos("siapAmbilGelas");
		pos("seduhKopi");
		Sleep(1000);
		mesinKopiOn();
		printf("mesin kopi nyala");
		Sleep(1000);
		printf("mesin kopi menunggu");

		gripGenggam();
		setKec(lambat);
		Sleep(1000);
		pos("siapSeduhKopi");
		pos("home");
	}

	setKec(lambat);
}

static void sajikanKopiKeStasiun(const char* poseStasiun, const char* poseStasiun2)
{
	persiapan();
	pos(poseStasiun2);
	pos(poseStasiun);
	gripLepas();
	Sleep(2000);
	pos(poseStasiun2);
	gripGenggam();
	Sleep(100);
	pos("home");
	gripLepas();
}

void siAPesenKopi()
{
	sajikanKopiKeStasiun("sajikanKopiA", "sajikanKopiA2");
}

void siBPesenKopi()
{
	persiapan();
	pos("sajikanKopiB2");
	Sleep(2000);
	pos("sajikanKopiB");
	gripLepas();
	Sleep(2000);
	pos("sajikanKopiB2");
	gripGenggam();
	Sleep(2000);
	pos("home");
	gripLepas();
}

void siCPesenKopi()
{
	persiapan();
	pos("sajikanKopiC2");
	Sleep(2000);
	pos("sajikanKopiC");
	gripLepas();
	Sleep(2000);
	pos("sajikanKopiC2");
	gripGenggam();
	Sleep(2000);
	pos("home");
	gripLepas();
}

static void jalankanUrutanEksekusi(const char* poseSajikan, const char* poseSajikan2)
{
	pos("siapAmbilGelas");
	pos("ambilGelas");
	gripGenggam();
	pos("siapSeduhKopi");
	pos("SeduhKopi");
	Sleep(10000);
	pos(poseSajikan);
	pos(poseSajikan2);
	Sleep(2000);
	gripLepas();
	Sleep(3000);
	pos("home");
}

static bool initGripper()
{
	if (Com1_Init(g_appConfig.comHand, 115200) != 1) {
		printf("Hands_Com open error...\n");
		return false;
	}

	printf("Tangan konek...\n");
	gripGenggam();
	Sleep(1000);
	gripLepas();
	Sleep(1000);
	printf("gripper setup\n");
	return true;
}

static bool initRobot()
{
	if (!example_login(g_rshd, g_appConfig.robotAddr.c_str(), g_appConfig.robotPort)) {
		printf("i5 not connect...\n");
		return false;
	}

	printf("i5 Connecting...\n");
	example_robotStartup(g_rshd);
	printf("setup\n");
	printf("aubo setup\n");
	printf(" setup done\n");
	return true;
}

static void jalankanSajikanStasiun(char stasiun)
{
	switch (stasiun) {
	case 'A':
	case 'a':
		jalankanUrutanEksekusi("sajikanKopiA", "sajikanKopiA2");
		break;
	case 'B':
	case 'b':
		jalankanUrutanEksekusi("sajikanKopiB", "sajikanKopiB2");
		break;
	case 'C':
	case 'c':
		jalankanUrutanEksekusi("sajikanKopiC", "sajikanKopiC2");
		break;
	default:
		printf("Stasiun tidak dikenal untuk sajikan: %c\n", stasiun);
		break;
	}
}

static void jalankanPenempatanStasiun(char stasiun)
{
	switch (stasiun) {
	case 'A':
	case 'a':
		siAPesenKopi();
		break;
	case 'B':
	case 'b':
		siBPesenKopi();
		break;
	case 'C':
	case 'c':
		siCPesenKopi();
		break;
	default:
		printf("Stasiun tidak dikenal untuk penempatan: %c\n", stasiun);
		break;
	}
}

static void mapIncomingMessageToCommands(const std::string& message)
{
	if (message.find("EKSEKUSI_1") != std::string::npos ||
		message.find("SAJIKAN_A") != std::string::npos) {
		enqueueRobotCommand("SAJIKAN_A");
	}
	if (message.find("EKSEKUSI_2") != std::string::npos ||
		message.find("SAJIKAN_B") != std::string::npos) {
		enqueueRobotCommand("SAJIKAN_B");
	}
	if (message.find("EKSEKUSI_3") != std::string::npos ||
		message.find("SAJIKAN_C") != std::string::npos) {
		enqueueRobotCommand("SAJIKAN_C");
	}
	if (message.find("PESAN_A") != std::string::npos ||
		message.find("PENEMPATAN_A") != std::string::npos) {
		enqueueRobotCommand("PESAN_A");
	}
	if (message.find("PESAN_B") != std::string::npos ||
		message.find("PENEMPATAN_B") != std::string::npos) {
		enqueueRobotCommand("PESAN_B");
	}
	if (message.find("PESAN_C") != std::string::npos ||
		message.find("PENEMPATAN_C") != std::string::npos) {
		enqueueRobotCommand("PESAN_C");
	}
}

static void executeRobotCommand(const std::string& command)
{
	printf("Menjalankan perintah: %s\n", command.c_str());

	if (command == "SAJIKAN_A") {
		jalankanSajikanStasiun('A');
	}
	else if (command == "SAJIKAN_B") {
		jalankanSajikanStasiun('B');
	}
	else if (command == "SAJIKAN_C") {
		jalankanSajikanStasiun('C');
	}
	else if (command == "PESAN_A") {
		jalankanPenempatanStasiun('A');
	}
	else if (command == "PESAN_B") {
		jalankanPenempatanStasiun('B');
	}
	else if (command == "PESAN_C") {
		jalankanPenempatanStasiun('C');
	}
	else {
		printf("Perintah tidak dikenal: %s\n", command.c_str());
	}
}

static DWORD WINAPI tcpBridgeThread(LPVOID)
{
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		printf("WSAStartup gagal\n");
		return 1;
	}

	SOCKET listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (listenSocket == INVALID_SOCKET) {
		printf("socket() gagal\n");
		WSACleanup();
		return 1;
	}

	BOOL reuseAddr = TRUE;
	setsockopt(listenSocket, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuseAddr, sizeof(reuseAddr));

	sockaddr_in serverAddr = {};
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	serverAddr.sin_port = htons(static_cast<u_short>(g_appConfig.tcpBridgePort));

	if (bind(listenSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
		printf("bind() port %d gagal\n", g_appConfig.tcpBridgePort);
		closesocket(listenSocket);
		WSACleanup();
		return 1;
	}

	if (listen(listenSocket, 5) == SOCKET_ERROR) {
		printf("listen() gagal\n");
		closesocket(listenSocket);
		WSACleanup();
		return 1;
	}

	printf("TCP bridge aktif di port %d (website Node.js)\n", g_appConfig.tcpBridgePort);
	if (!g_appConfig.apiToken.empty()) {
		printf("TCP bridge: autentikasi token AKTIF\n");
	}

	while (g_appRunning) {
		fd_set readSet;
		FD_ZERO(&readSet);
		FD_SET(listenSocket, &readSet);

		timeval timeout = {};
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;

		const int ready = select(0, &readSet, NULL, NULL, &timeout);
		if (ready <= 0) {
			continue;
		}

		SOCKET clientSocket = accept(listenSocket, NULL, NULL);
		if (clientSocket == INVALID_SOCKET) {
			continue;
		}

		char buffer[256] = {};
		const int received = recv(clientSocket, buffer, sizeof(buffer) - 1, 0);
		if (received > 0) {
			std::string message(buffer, received);
			std::string payload;
			if (!isTcpMessageAuthorized(message, payload)) {
				const char* response = "UNAUTHORIZED\n";
				send(clientSocket, response, (int)strlen(response), 0);
			}
			else {
				mapIncomingMessageToCommands(payload);
				const char* response = "OK\n";
				send(clientSocket, response, (int)strlen(response), 0);
			}
		}

		closesocket(clientSocket);
	}

	closesocket(listenSocket);
	WSACleanup();
	return 0;
}

static bool startTcpBridge()
{
	const HANDLE threadHandle = CreateThread(NULL, 0, tcpBridgeThread, NULL, 0, NULL);
	if (threadHandle == NULL) {
		printf("Gagal memulai thread TCP bridge\n");
		return false;
	}
	CloseHandle(threadHandle);
	return true;
}

static int runRobotControlLoop()
{
	serial::Serial* arduinoSerial = NULL;
	try {
		arduinoSerial = new serial::Serial(
			g_appConfig.arduinoPort,
			g_appConfig.arduinoBaud,
			serial::Timeout::simpleTimeout(1000));
		printf("Arduino serial %s @ %lu (opsional)\n",
			g_appConfig.arduinoPort.c_str(),
			g_appConfig.arduinoBaud);
	}
	catch (const std::exception& e) {
		printf("Serial Arduino tidak terbuka (%s). Website TCP tetap jalan.\n", e.what());
	}

	while (g_appRunning) {
		std::string command;
		if (dequeueRobotCommand(command)) {
			executeRobotCommand(command);
			continue;
		}

		if (arduinoSerial != NULL && arduinoSerial->isOpen() && arduinoSerial->available()) {
			const std::string data_received = arduinoSerial->read(100);
			std::cout << "Arduino: " << data_received << std::endl;
			mapIncomingMessageToCommands(data_received);
		}

		Sleep(50);
	}

	delete arduinoSerial;
	return 0;
}

int _tmain(int argc, _TCHAR* argv[])
{
	(void)argc;
	(void)argv;

	printf("cikalong");

	if (!loadRobotAppConfig("robot.config.ini")) {
		system("Pause");
		return 0;
	}

	if (!initGripper()) {
		system("Pause");
		return 0;
	}

	if (!initRobot()) {
		system("Pause");
		return 0;
	}

	if (!startTcpBridge()) {
		system("Pause");
		return 0;
	}

	printf("Jalankan website: cd web-control && npm install && npm start\n");
	printf("Buka http://localhost:3000\n");

	runRobotControlLoop();

	g_appRunning = false;
	rs_uninitialize();

	std::cout << "Pencet ENTER untuk keluar" << std::endl;
	getchar();
	return 0;
}
