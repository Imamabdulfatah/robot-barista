#pragma once

#include <string>
#include "rsdef.h"

using namespace std;


extern double User_jointMaxAcc[6];
extern double User_jointMaxVelc[6];


//打印路点信息
void printRoadPoint(const aubo_robot_namespace::wayPoint_S  *wayPoint);

void callback_RealTimeRoadPoint(const aubo_robot_namespace::wayPoint_S  *wayPoint, void *arg);

//登陆机械臂
bool example_login(RSHD &rshd, const char * addr, int port);

//退出登陆
bool example_logout(RSHD rshd);

//启动机械臂(必须连接真实机械臂）
bool example_robotStartup(RSHD rshd);

//关闭机械臂（必须连接真实机械臂）
bool example_robotShutdown(RSHD rshd);

//机械臂关节角运动，需要先设置加速度和速度
bool JXB_move_SetPara(RSHD rshd);
bool JXB_move_GoToJoint(RSHD rshd, double Ang1, double Ang2, double Ang3, double Ang4, double Ang5, double Ang6);

bool move_to(RSHD rshd, const Pos *pos, double joint6Angle);




//机械臂轴动测试
bool example_moveJ(RSHD rshd);

//机械臂保持当前姿态直线运动测试
bool example_moveL(RSHD rshd);

//机械臂轨迹运动测试
void example_moveP(RSHD rshd);

//机械臂正逆解测试
void example_ik_fk(RSHD rshd);

//机械臂控制柜IO测试(必须连接真实机械臂）
bool example_boardInput(RSHD rshd);
void example_boardOutput(RSHD rshd);
void example_boardIO(RSHD rshd);
bool pin_DI_00(RSHD rshd);
bool pin_DI_01(RSHD rshd);
bool pin_DI_02(RSHD rshd);
bool pin_DI_03(RSHD rshd);
bool pin_DI_04(RSHD rshd);
bool pin_DI_05(RSHD rshd);
bool pin_DI_06(RSHD rshd);
bool pin_DI_07(RSHD rshd);
bool pin_DI_17(RSHD rshd);

void pin_D0_00_High(RSHD rshd);
void pin_D0_01_High(RSHD rshd);
void pin_D0_02_High(RSHD rshd);
void pin_D0_03_High(RSHD rshd);
void pin_D0_04_High(RSHD rshd);
void pin_D0_05_High(RSHD rshd);
void pin_D0_06_High(RSHD rshd);
void pin_D0_07_High(RSHD rshd);

void pin_D0_00_Low(RSHD rshd);
void pin_D0_01_Low(RSHD rshd);
void pin_D0_02_Low(RSHD rshd);
void pin_D0_03_Low(RSHD rshd);
void pin_D0_04_Low(RSHD rshd);
void pin_D0_05_Low(RSHD rshd);
void pin_D0_06_Low(RSHD rshd);
void pin_D0_07_Low(RSHD rshd);




//机械臂工具端IO测试(必须连接真实机械臂）
void example_ToolIO(RSHD rshd);

//实时路点信息回调函数测试
bool example_callbackRobotRoadPoint(RSHD rshd);

//串口初始化配置，遍历串口号，同时打开合适的串口
int Com1_Init(int comport, DWORD baud_rate);
int Com2_Init(int comport, DWORD baud_rate);
int Com_Hands_SpeedSet(int data1, int data2, int data3, int data4, int data5, int data6);
int Com_Hands_AngleSet(int data1, int data2, int data3, int data4, int data5, int data6);
int Com_Hands_LiKongSet(int data1, int data2, int data3, int data4, int data5);
int Uart1_Send_Array(unsigned int Len);

int Uart1_Rec(void);
bool Uart1_MC_JieXi(void);
void MC1_Rec_Pro(void);
int Uart2_Rec(void);
bool Uart2_MC_JieXi(void);
void MC2_Rec_Pro(void);
int Uart2_Send_Array(unsigned int Len);
int Com_Btn_LED_Set(int data1);




extern unsigned char CMD1_MC_Array[50];
extern unsigned char Com1_RecBuf[50];
extern unsigned char CMD1_MC_REC_Array[50];
extern unsigned char CMD2_MC_Array[50];
extern unsigned char Com2_RecBuf[50];
extern unsigned char CMD2_MC_REC_Array[50];

#define  Hands_ID  1


#define FRAME_HEAD1  				0xEB
#define FRAME_HEAD2  				0x90
#define FRAME_HEAD1_RT  			0xEE
#define FRAME_HEAD2_RT  			0x16

#define CMD_MC_SET_DRVALL_SEEKPOS 							0x50
#define CMD_MC_SET_DRVALL_SPEED 							0x51
#define CMD_MC_SET_DRVALL_YBP_THRESHOLD						0x52
#define CMD_MC_SET_DRVALL_SEEKANGLE							0x53
#define CMD_MC_SET_DRVALL_SEEKANGLE_GYH						0x54
#define CMD_MC_SET_DJ_TURN									0x55


#define CMD_MC_READ_DRVALL_SEEKPOS 							0xD0
#define CMD_MC_READ_DRVALL_CURPOS 							0xD1
#define CMD_MC_READ_DRVALL_SPEED 							0xD2
#define CMD_MC_READ_DRVALL_YBP_THRESHOLD					0xD3
#define CMD_MC_READ_DRVALL_YBP_RAWDATA						0xD4
#define CMD_MC_READ_DRVALL_SEEKANGLE 						0xD5
#define CMD_MC_READ_DRVALL_CURANGLE 						0xD6
#define CMD_MC_READ_DRVALL_SEEKGYHANGLE 					0xD7
#define CMD_MC_READ_DJ_TURN									0xD8
#define CMD_MC_READ_DRVALL_CURANGLE_GYH						0xD9
#define CMD_MC_READ_BIT										0xDA


#define CMD_MC_PARA_SAVE 											0x01
#define CMD_MC_PARA_READ 											0x02
#define CMD_MC_PARA_USED_DEF										0x03
#define CMD_MC_PARA_ID_SET											0x04
#define CMD_MC_MOVE_K_SET_CURPOS_IN_CMAP							0x05
#define CMD_MC_MOVE_K_SHOW											0x06
#define CMD_MC_MOVE_K_SHOW_ASDEFAULT								0x07