#include "stdafx.h"
#include "example.h"
#include "IO.h"
#include "windows.h"

aubo_robot_namespace::JointVelcAccParam JXB_jointMaxAcc;
aubo_robot_namespace::JointVelcAccParam JXB_jointMaxVelc;
double JXB_endMoveMaxAcc = 0.2;		// awalnya 0.2
double JXB_endMoveMaxVelc = 1;	// awalnya 0.2

HANDLE hCom1;
OVERLAPPED m_osWrite1;
FILE *fp_com1;

HANDLE hCom2;
OVERLAPPED m_osWrite2;

unsigned char CMD1_MC_Array[50] = { 0 };
unsigned char Com1_RecBuf[50] = { 0 };
unsigned char CMD1_MC_REC_Array[50] = { 0 };
int Send1_Len = 0;
int Rec1_Len = 0;

unsigned char CMD2_MC_Array[50] = { 0 };
unsigned char Com2_RecBuf[50] = { 0 };
unsigned char CMD2_MC_REC_Array[50] = { 0 };
int Send2_Len = 0;
int Rec2_Len = 0;
int State_Btn = 0;

int State_Led = 0;
#define M_PI 3.14159265358979323846

//控制柜用户ＤＩ名称
const char* USER_DI_00 = "U_DI_00";
const char* USER_DI_01 = "U_DI_01";
const char* USER_DI_02 = "U_DI_02";
const char* USER_DI_03 = "U_DI_03";
const char* USER_DI_04 = "U_DI_04";
const char* USER_DI_05 = "U_DI_05";
const char* USER_DI_06 = "U_DI_06";
const char* USER_DI_07 = "U_DI_07";
const char* USER_DI_10 = "U_DI_10";
const char* USER_DI_11 = "U_DI_11";
const char* USER_DI_12 = "U_DI_12";
const char* USER_DI_13 = "U_DI_13";
const char* USER_DI_14 = "U_DI_14";
const char* USER_DI_15 = "U_DI_15";
const char* USER_DI_16 = "U_DI_16";
const char* USER_DI_17 = "U_DI_17";

//控制柜用户ＤＯ名称
const char* USER_DO_00 = "U_DO_00";
const char* USER_DO_01 = "U_DO_01";
const char* USER_DO_02 = "U_DO_02";
const char* USER_DO_03 = "U_DO_03";
const char* USER_DO_04 = "U_DO_04";
const char* USER_DO_05 = "U_DO_05";
const char* USER_DO_06 = "U_DO_06";
const char* USER_DO_07 = "U_DO_07";
const char* USER_DO_10 = "U_DO_10";
const char* USER_DO_11 = "U_DO_11";
const char* USER_DO_12 = "U_DO_12";
const char* USER_DO_13 = "U_DO_13";
const char* USER_DO_14 = "U_DO_14";
const char* USER_DO_15 = "U_DO_15";
const char* USER_DO_16 = "U_DO_16";
const char* USER_DO_17 = "U_DO_17";

const char* TOOL_IO_0 = "T_DI/O_00";
const char* TOOL_IO_1 = "T_DI/O_01"	;
const char* TOOL_IO_2 = "T_DI/O_02"	;
const char* TOOL_IO_3 = "T_DI/O_03"	;


void printRoadPoint(const aubo_robot_namespace::wayPoint_S  *wayPoint)
{
	/*
	std::cout<<"pos.x="<<wayPoint->cartPos.position.x<<std::endl;
	std::cout<<"pos.y="<<wayPoint->cartPos.position.y<<std::endl;
	std::cout<<"pos.z="<<wayPoint->cartPos.position.z<<std::endl;

	std::cout<<"ori.w="<<wayPoint->orientation.w<<std::endl;
	std::cout<<"ori.x="<<wayPoint->orientation.x<<std::endl;
	std::cout<<"ori.y="<<wayPoint->orientation.y<<std::endl;
	std::cout<<"ori.z="<<wayPoint->orientation.z<<std::endl;

	std::cout<<"joint_1="<<wayPoint->jointpos[0]*180.0/M_PI<<std::endl;
	std::cout<<"joint_2="<<wayPoint->jointpos[1]*180.0/M_PI<<std::endl;
	std::cout<<"joint_3="<<wayPoint->jointpos[2]*180.0/M_PI<<std::endl;
	std::cout<<"joint_4="<<wayPoint->jointpos[3]*180.0/M_PI<<std::endl;
	std::cout<<"joint_5="<<wayPoint->jointpos[4]*180.0/M_PI<<std::endl;
	std::cout<<"joint_6="<<wayPoint->jointpos[5]*180.0/M_PI<<std::endl;
	*/

	std::cout << "joint_1 sampai joint_6 = " << wayPoint->jointpos[0] * 180.0 / M_PI << ", ";
	std::cout << wayPoint->jointpos[1] * 180.0 / M_PI << ", ";
	std::cout << wayPoint->jointpos[2] * 180.0 / M_PI << ", ";
	std::cout << wayPoint->jointpos[3] * 180.0 / M_PI << ", ";
	std::cout << wayPoint->jointpos[4] * 180.0 / M_PI << ", ";
	std::cout << wayPoint->jointpos[5] * 180.0 / M_PI << std::endl << std::endl;
}

void callback_RealTimeRoadPoint(const aubo_robot_namespace::wayPoint_S  *wayPoint, void *arg)
{
	printRoadPoint(wayPoint);
}

/************************************************************************/
/* 
   pos 目标位置x,y,z 单位米
   joint6Angle 6轴角度(度)
*/
/************************************************************************/
bool move_to(RSHD rshd, const Pos *pos, double joint6Angle)
{
	bool result = false;

	//首先获取当前路点信息
	aubo_robot_namespace::wayPoint_S wayPoint;

	//逆解位置信息
	aubo_robot_namespace::wayPoint_S targetPoint;

	//目标位置对应的关节角
	double targetRadian[ARM_DOF] = {0};
	
	if (RS_SUCC == rs_get_current_waypoint(rshd, &wayPoint))
	{
		//参考当前姿态逆解得到六个关节角
		if (RS_SUCC == rs_inverse_kin(rshd, wayPoint.jointpos, pos, &wayPoint.orientation, &targetPoint))
		{
			//将得到目标位置,将6关节角度设置为用户给定的角度（必须在+-175度）
			targetRadian[0] = targetPoint.jointpos[0];
			targetRadian[1] = targetPoint.jointpos[1];
			targetRadian[2] = targetPoint.jointpos[2];
			targetRadian[3] = targetPoint.jointpos[3];
			targetRadian[4] = targetPoint.jointpos[4];
			//6轴使用用户给定的关节角度
			targetRadian[5] = joint6Angle/180*M_PI;

			//轴动到目标位置
			if (RS_SUCC == rs_move_joint(rshd, targetRadian))
			{
				std::cout<<"到达目标位置"<<std::endl;

				//获取当前关节角，进行验证
				rs_get_current_waypoint(rshd, &wayPoint);

				printRoadPoint(&wayPoint);
			}
			else
			{
				std::cerr<<"move joint error"<<std::endl;
			}
		}
		else
		{
			std::cerr<<"ik failed"<<std::endl;
		}

	}
	else
	{
		std::cerr<<"get current waypoint error"<<std::endl;
	}

	return result;
}

/********************************************************************
	function:	example_login
	purpose :	登陆机械臂
	param   :	rshd 输出上下文句柄
				addr 机械臂服务器地址
				port 机械臂服务器端口
	return  :	true 成功 false 失败
*********************************************************************/
bool example_login(RSHD &rshd, const char * addr, int port)
{
	bool result = false;

	rshd = RS_FAILED;

	//初始化接口库
	if (rs_initialize() == RS_SUCC)
	{
		//创建上下文
		if (rs_create_context(&rshd)  == RS_SUCC )
		{
			//登陆机械臂服务器
			if (rs_login(rshd, addr, port) == RS_SUCC)
			{
				result = true;
				//登陆成功
				std::cout<<"login succ"<<std::endl;
			}
			else
			{
				//登陆失败
				std::cerr<<"login failed"<<std::endl;				
			}
		}
		else
		{
			//创建上下文失败
			std::cerr<<"rs_create_context error"<<std::endl;
		}
	}
	else
	{
		//初始化接口库失败
		std::cerr<<"rs_initialize error"<<std::endl;
	}

	return result;
}

/********************************************************************
	function:	example_logout
	purpose :	退出登陆
	param   :	rshd 上下文句柄
					
	return  :	true 成功 false 失败
*********************************************************************/
bool example_logout(RSHD rshd)
{
	return rs_logout(rshd)==RS_SUCC ? true : false;
}

/********************************************************************
	function:	example_robotStartup
	purpose :	启动机械臂(必须连接真实机械臂）
	param   :	rshd 上下文句柄
					
	return  :	true 成功 false 失败
*********************************************************************/
bool example_robotStartup(RSHD rshd)
{
	bool result = false;

	//工具的动力学参数和运动学参数
	ToolDynamicsParam tool_dynamics = {0};
	//机械臂碰撞等级
	uint8 colli_class = 8;
	//机械臂启动是否读取姿态（默认开启）
	bool read_pos = true;
	//机械臂静态碰撞检测（默认开启）
	bool static_colli_detect = true;
	//机械臂最大加速度（系统自动控制，默认为30000)
	int board_maxacc = 3000;
	//机械臂服务启动状态
	ROBOT_SERVICE_STATE state = ROBOT_SERVICE_READY;

	if (rs_robot_startup(rshd, &tool_dynamics, colli_class, read_pos, static_colli_detect, board_maxacc, &state)
		== RS_SUCC)
	{
		result = true;
		std::cout<<"call robot startup succ, robot state:"<<state<<std::endl;
	}
	else
	{
		std::cerr<<"robot startup failed"<<std::endl;
	}

	return result;
}

/********************************************************************
	function:	example_robotShutdown
	purpose :	关闭机械臂（必须连接真实机械臂）
	param   :	rshd 上下文句柄
					
	return  :	true 成功 false 失败
*********************************************************************/
bool example_robotShutdown(RSHD rshd)
{
	return rs_robot_shutdown(rshd)==RS_SUCC ? true : false;
}
/********************************************************************
function:	JXB_move_SetPara
purpose :	机械臂设置加速度和速度
param   :	rshd 上下文句柄

return  :	true 成功 false 失败
*********************************************************************/
bool JXB_move_SetPara(RSHD rshd)
{
	bool result = false;
	/** 模拟业务 **/
	/** 接口调用: 初始化运动属性 ***/
	rs_init_global_move_profile(rshd);

	/** 接口调用: 设置关节型运动的最大加速度 ***/
	
	JXB_jointMaxAcc.jointPara[0] = User_jointMaxAcc[0] / 180.0*M_PI;
	JXB_jointMaxAcc.jointPara[1] = User_jointMaxAcc[1] / 180.0*M_PI;
	JXB_jointMaxAcc.jointPara[2] = User_jointMaxAcc[2] / 180.0*M_PI;
	JXB_jointMaxAcc.jointPara[3] = User_jointMaxAcc[3] / 180.0*M_PI;
	JXB_jointMaxAcc.jointPara[4] = User_jointMaxAcc[4] / 180.0*M_PI;
	JXB_jointMaxAcc.jointPara[5] = User_jointMaxAcc[5] / 180.0*M_PI;   //接口要求单位是弧度
	rs_set_global_joint_maxacc(rshd, &JXB_jointMaxAcc);

	/** 接口调用: 设置关节型运动的最大速度 ***/
	
	JXB_jointMaxVelc.jointPara[0] = User_jointMaxVelc[0] / 180.0*M_PI;
	JXB_jointMaxVelc.jointPara[1] = User_jointMaxVelc[1] / 180.0*M_PI;
	JXB_jointMaxVelc.jointPara[2] = User_jointMaxVelc[2] / 180.0*M_PI;
	JXB_jointMaxVelc.jointPara[3] = User_jointMaxVelc[3] / 180.0*M_PI;
	JXB_jointMaxVelc.jointPara[4] = User_jointMaxVelc[4] / 180.0*M_PI;
	JXB_jointMaxVelc.jointPara[5] = User_jointMaxVelc[5] / 180.0*M_PI;   //接口要求单位是弧度
	rs_set_global_joint_maxvelc(rshd, &JXB_jointMaxVelc);


	/** 接口调用: 初始化运动属性 ***/
	rs_init_global_move_profile(rshd);

	/** 接口调用: 设置末端型运动的最大加速度 　　直线运动属于末端型运动***/
	 //单位米每秒 0.2
	rs_set_global_end_max_line_acc(rshd,  JXB_endMoveMaxAcc);
	rs_set_global_end_max_angle_acc(rshd, JXB_endMoveMaxAcc);


	/** 接口调用: 设置末端型运动的最大速度 直线运动属于末端型运动***/
	//单位米每秒 0.2
	rs_set_global_end_max_line_velc(rshd,  JXB_endMoveMaxVelc);
	rs_set_global_end_max_angle_velc(rshd, JXB_endMoveMaxVelc);

	return result;
}

/********************************************************************
function:	JXB_move_GoToJoint
purpose :	机械臂运动到零位姿态
param   :	rshd 上下文句柄

return  :	true 成功 false 失败
*********************************************************************/
bool JXB_move_GoToJoint(RSHD rshd,double Ang1,double Ang2,double Ang3,double Ang4,double Ang5,double Ang6)
{
	bool result = false;
	//该位置为机械臂的初始位置（提供6个关节角的关节信息（单位：弧度））pos=[0,-0.213,0.782] atti=[89.99,0,0]
	double initPos[6] = {
		Ang1 / 180 * M_PI,
		Ang2 / 180 * M_PI,
		Ang3 / 180 * M_PI,
		Ang4 / 180 * M_PI,
		Ang5 / 180 * M_PI,
		Ang6 / 180 * M_PI };

	rs_init_global_move_profile(rshd);
	rs_set_global_joint_maxacc(rshd, &JXB_jointMaxAcc);
	rs_set_global_joint_maxvelc(rshd, &JXB_jointMaxVelc);

	//首先运动到初始位置
	if (rs_move_joint(rshd, initPos) == RS_SUCC)
	{
		result = true;
		std::cout << "movej succ" << std::endl;
	}
	else
	{
		std::cerr << "movej failed!" << std::endl;
	}
	//std::cout << "Step Debug Next???" << std::endl; 
	//getchar();
	return result;
}
/********************************************************************
	function:	example_moveJ
	purpose :	机械臂轴动测试
	param   :	rshd 上下文句柄
					
	return  :	true 成功 false 失败
*********************************************************************/
bool example_moveJ(RSHD rshd)
{
	bool result = false;

	//该位置为机械臂的初始位置（提供6个关节角的关节信息（单位：弧度））
	double initPos[6]={
		-0.000172/180*M_PI,
		-7.291862/180*M_PI,
		-75.694718/180*M_PI,
		21.596727/180*M_PI,
		-89.999982/180*M_PI,
		-0.00458/180*M_PI};

	//首先运动到初始位置
	if (rs_move_joint(rshd, initPos) == RS_SUCC)
	{
		result = true;
		std::cout<<"movej succ"<<std::endl;
	}
	else
	{
		std::cerr<<"movej failed!"<<std::endl;
	}

	return result;
}
/********************************************************************
function:	JXB_moveJ
purpose :	机械臂轴动测试
param   :	rshd 上下文句柄

return  :	true 成功 false 失败
*********************************************************************/
bool JXB_moveJ(RSHD rshd,double ang1,double ang2,double ang3,double ang4,double ang5,double ang6)
{
	bool result = false;

	//该位置为机械臂的初始位置（提供6个关节角的关节信息（单位：弧度））
	double initPos[6] = {
		ang1 / 180 * M_PI,
		ang2 / 180 * M_PI,
		ang3 / 180 * M_PI,
		ang4 / 180 * M_PI,
		ang5 / 180 * M_PI,
		ang6 / 180 * M_PI };

	//首先运动到初始位置
	if (rs_move_joint(rshd, initPos) == RS_SUCC)
	{
		result = true;
		std::cout << "movej succ" << std::endl;
	}
	else
	{
		std::cerr << "movej failed!" << std::endl;
	}

	return result;
}
/********************************************************************
	function:	example_moveL
	purpose :	机械臂保持当前姿态直线运动测试
	param   :	rshd 上下文句柄
					
	return  :	true 成功 false 失败
*********************************************************************/
bool example_moveL(RSHD rshd)
{
	bool result = false;

	//首先移动到初始位置
	// example_moveJ(rshd);

	//获取当前路点信息
	aubo_robot_namespace::wayPoint_S wayPoint;

	//逆解位置信息
	aubo_robot_namespace::wayPoint_S targetPoint;

	//目标位置对应的关节角
	double targetRadian[ARM_DOF] = {0};

	//目标位置
	Pos pos = {-0.489605, -0.155672, 0.448430};

	if (RS_SUCC == rs_get_current_waypoint(rshd, &wayPoint))
	{
		//参考当前姿态逆解得到六个关节角
		if (RS_SUCC == rs_inverse_kin(rshd, wayPoint.jointpos, &pos, &wayPoint.orientation, &targetPoint))
		{
			//将得到目标位置,将6关节角度设置为用户给定的角度（必须在+-175度）
			targetRadian[0] = targetPoint.jointpos[0];
			targetRadian[1] = targetPoint.jointpos[1];
			targetRadian[2] = targetPoint.jointpos[2];
			targetRadian[3] = targetPoint.jointpos[3];
			targetRadian[4] = targetPoint.jointpos[4];
			targetRadian[5] = targetPoint.jointpos[5];

			//轴动到目标位置
			if (RS_SUCC == rs_move_line(rshd, targetRadian))
			{
				std::cout<<"at target"<<std::endl;
			}
			else
			{
				std::cerr<<"move joint error"<<std::endl;
			}
		}
		else
		{
			std::cerr<<"ik failed"<<std::endl;
		}

	}
	else
	{
		std::cerr<<"get current waypoint error"<<std::endl;
	}


	return result;
}

/********************************************************************
	function:	example_moveP
	purpose :	机械臂轨迹运动测试
	param   :	rshd 上下文句柄
					
	return  :	void
*********************************************************************/
void example_moveP(RSHD rshd)
{
	/** 模拟业务 **/
	/** 接口调用: 初始化运动属性 ***/
	rs_init_global_move_profile(rshd);

	/** 接口调用: 设置关节型运动的最大加速度 ***/
	aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
	jointMaxAcc.jointPara[0] = 50.0/180.0*M_PI;
	jointMaxAcc.jointPara[1] = 50.0/180.0*M_PI;
	jointMaxAcc.jointPara[2] = 50.0/180.0*M_PI;
	jointMaxAcc.jointPara[3] = 50.0/180.0*M_PI;
	jointMaxAcc.jointPara[4] = 50.0/180.0*M_PI;
	jointMaxAcc.jointPara[5] = 50.0/180.0*M_PI;   //接口要求单位是弧度
	rs_set_global_joint_maxacc(rshd, &jointMaxAcc);

	/** 接口调用: 设置关节型运动的最大速度 ***/
	aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
	jointMaxVelc.jointPara[0] = 50.0/180.0*M_PI;
	jointMaxVelc.jointPara[1] = 50.0/180.0*M_PI;
	jointMaxVelc.jointPara[2] = 50.0/180.0*M_PI;
	jointMaxVelc.jointPara[3] = 50.0/180.0*M_PI;
	jointMaxVelc.jointPara[4] = 50.0/180.0*M_PI;
	jointMaxVelc.jointPara[5] = 50.0/180.0*M_PI;   //接口要求单位是弧度
	rs_set_global_joint_maxvelc(rshd, &jointMaxVelc);


	/** 接口调用: 初始化运动属性 ***/
	rs_init_global_move_profile(rshd);

	/** 接口调用: 设置末端型运动的最大加速度 　　直线运动属于末端型运动***/
	double endMoveMaxAcc;
	endMoveMaxAcc = 0.2;   //单位米每秒
	rs_set_global_end_max_line_acc(rshd, endMoveMaxAcc);
	rs_set_global_end_max_angle_acc(rshd, endMoveMaxAcc);


	/** 接口调用: 设置末端型运动的最大速度 直线运动属于末端型运动***/
	double endMoveMaxVelc;
	endMoveMaxVelc = 0.2;   //单位米每秒
	rs_set_global_end_max_line_velc(rshd, endMoveMaxVelc);
	rs_set_global_end_max_angle_velc(rshd, endMoveMaxVelc);

	double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};

	for(int i=0;i<2;i++)
	{
		//准备点  关节运动属于关节型运动
		rs_init_global_move_profile(rshd);
		rs_set_global_joint_maxacc(rshd, &jointMaxAcc);
		rs_set_global_joint_maxvelc(rshd, &jointMaxVelc);

		//关节运动至准备点
		jointAngle[0] = -0.000003;
		jointAngle[1] = -0.127267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570796;
		jointAngle[5] = -0.000008;

		int ret = rs_move_joint(rshd, jointAngle);
		if(ret != RS_SUCC)
		{
			std::cerr<<"JointMove失败.　ret:"<<ret<<std::endl;
		}

		//圆弧
		rs_init_global_move_profile(rshd);

		rs_set_global_end_max_line_acc(rshd, endMoveMaxAcc);
		rs_set_global_end_max_angle_acc(rshd, endMoveMaxAcc);
		rs_set_global_end_max_line_velc(rshd, endMoveMaxVelc);
		rs_set_global_end_max_angle_velc(rshd, endMoveMaxVelc);

		jointAngle[0] = -0.000003;
		jointAngle[1] = -0.127267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570796;
		jointAngle[5] = -0.000008;
		rs_add_waypoint(rshd, jointAngle);

		jointAngle[0] = 0.200000;
		jointAngle[1] = -0.127267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570794;
		jointAngle[5] = -0.000008;
		rs_add_waypoint(rshd, jointAngle);

		jointAngle[0] = 0.600000;
		jointAngle[1] = -0.127267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570796;
		jointAngle[5] = -0.000008;
		rs_add_waypoint(rshd, jointAngle);

		rs_set_circular_loop_times(rshd, 0);
		if(RS_SUCC !=rs_move_track(rshd, ARC_CIR))
		{
			std::cerr<<"TrackMove failed.　ret:"<<ret<<std::endl;
		}

		//准备点
		rs_init_global_move_profile(rshd);
		rs_set_global_joint_maxacc(rshd, &jointMaxAcc);
		rs_set_global_joint_maxvelc(rshd, &jointMaxVelc);

		jointAngle[0] = -0.000003;
		jointAngle[1] = -0.127267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570796;
		jointAngle[5] = -0.000008;

		//关节运动至准备点
		ret = rs_move_joint(rshd, jointAngle);
		if(RS_SUCC != ret)
		{
			std::cerr<<"JointMove失败.　ret:"<<ret<<std::endl;
		}

		//圆
		rs_init_global_move_profile(rshd);

		rs_set_global_end_max_line_acc(rshd, endMoveMaxAcc);
		rs_set_global_end_max_angle_acc(rshd, endMoveMaxAcc);
		rs_set_global_end_max_line_velc(rshd, endMoveMaxVelc);
		rs_set_global_end_max_angle_velc(rshd, endMoveMaxVelc);

		jointAngle[0] = -0.000003;
		jointAngle[1] = -0.127267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570796;
		jointAngle[5] = -0.000008;
		rs_add_waypoint(rshd, jointAngle);

		jointAngle[0] = -0.211675;
		jointAngle[1] = -0.325189;
		jointAngle[2] = -1.466753;
		jointAngle[3] = 0.429232;
		jointAngle[4] = -1.570794;
		jointAngle[5] = -0.211680;
		rs_add_waypoint(rshd, jointAngle);

		jointAngle[0] = -0.037186;
		jointAngle[1] = -0.224307;
		jointAngle[2] = -1.398285;
		jointAngle[3] = 0.396819;
		jointAngle[4] = -1.570796;
		jointAngle[5] = -0.037191;
		rs_add_waypoint(rshd, jointAngle);

		//圆的圈数
		rs_set_circular_loop_times(rshd, 1);
		ret = rs_move_track(rshd, ARC_CIR);
		if(RS_SUCC != ret)
		{
			std::cerr<<"TrackMove failed.　ret:"<<ret<<std::endl;
		}


		//准备点
		rs_init_global_move_profile(rshd);

		rs_set_global_joint_maxacc(rshd, &jointMaxAcc);
		rs_set_global_joint_maxvelc(rshd, &jointMaxVelc);

		jointAngle[0] = -0.000003;
		jointAngle[1] = -0.127267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570796;
		jointAngle[5] = -0.000008;

		//关节运动至准备点
		if(RS_SUCC != rs_move_joint(rshd, jointAngle))
		{
			std::cerr<<"JointMove失败.　ret:"<<ret<<std::endl;
		}

		//MoveP
		rs_init_global_move_profile(rshd);

		rs_set_global_end_max_line_acc(rshd, endMoveMaxAcc);
		rs_set_global_end_max_angle_acc(rshd, endMoveMaxAcc);
		rs_set_global_end_max_line_velc(rshd, endMoveMaxVelc);
		rs_set_global_end_max_angle_velc(rshd, endMoveMaxVelc);


		jointAngle[0] = -0.000003;
		jointAngle[1] = -0.127267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570796;
		jointAngle[5] = -0.000008;
		rs_add_waypoint(rshd, jointAngle);

		jointAngle[0] = 0.100000;
		jointAngle[1] = -0.147267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570794;
		jointAngle[5] = -0.000008;
		rs_add_waypoint(rshd, jointAngle);

		jointAngle[0] = 0.200000;
		jointAngle[1] = -0.167267;
		jointAngle[2] = -1.321122;
		jointAngle[3] = 0.376934;
		jointAngle[4] = -1.570794;
		jointAngle[5] = -0.000008;
		rs_add_waypoint(rshd, jointAngle);

		//交融半径
		rs_set_blend_radius(rshd, 0.03);
		rs_set_circular_loop_times(rshd, 1);
		if(RS_SUCC !=rs_move_track(rshd, CARTESIAN_MOVEP))
		{
			std::cerr<<"TrackMove failed.　ret:"<<ret<<std::endl;
		}
	}
}

/********************************************************************
	function:	example_ik_fk
	purpose :	机械臂正逆解测试
	param   :	
					
	return  :	
*********************************************************************/
void example_ik_fk(RSHD rshd)
{
	aubo_robot_namespace::wayPoint_S wayPoint;

	double jointAngle[aubo_robot_namespace::ARM_DOF] = {-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008};
	
	//正解
	if (RS_SUCC == rs_forward_kin(rshd, jointAngle, &wayPoint))
	{
		std::cout<<"fk succ"<<std::endl;

		printRoadPoint(&wayPoint);
	}

	//逆解
	double startPointJointAngle[aubo_robot_namespace::ARM_DOF] = {0.0/180.0*M_PI,  0.0/180.0*M_PI,  0.0/180.0*M_PI, 0.0/180.0*M_PI, 0.0/180.0*M_PI,0.0/180.0*M_PI};

	aubo_robot_namespace::Pos targetPosition;
	targetPosition.x =-0.400;
	targetPosition.y =-0.1215;
	targetPosition.z = 0.5476;

	aubo_robot_namespace::Rpy rpy;
	aubo_robot_namespace::Ori targetOri;

	rpy.rx = 180.0/180.0*M_PI;
	rpy.ry = 0.0/180.0*M_PI;
	rpy.rz = -90.0/180.0*M_PI;

	rs_rpy_to_quaternion(rshd, &rpy, &targetOri);

	if (RS_SUCC == rs_inverse_kin(rshd, startPointJointAngle, &targetPosition, &targetOri, &wayPoint))
	{
		std::cout<<"ik succ"<<std::endl;
		printRoadPoint(&wayPoint);
	}
	else
	{
		std::cerr<<"ik failed"<<std::endl;
	}
}

/********************************************************************
	function:	example_boardIO
	purpose :	机械臂控制柜IO测试(必须连接真实机械臂）
	param   :	rshd 上下文句柄
					
	return  :	void
*********************************************************************/
bool example_boardInput(RSHD rshd)
{
	double status = 0;
	if (RS_SUCC == rs_get_board_io_status_by_name(rshd, RobotBoardUserDI, USER_DI_00, &status))
	{
		//std::cout<<"get "<<USER_DI_00<<"="<< statusInput <<std::endl;
		return status;
	}
	else
	{
		//std::cerr<<"get "<<USER_DI_00<<" failed"<<std::endl;
		return false;
	}
}

void example_boardOutput(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_00, IO_STATUS_VALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << " set " << USER_DO_00 << " succ" << std::endl;
	}
	else
	{
		std::cerr << " set " << USER_DO_00 << " failed" << std::endl;
	}

}

void example_boardIO(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_00, IO_STATUS_INVALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << "set " << USER_DO_00 << " succ" << std::endl;

		// Cek apakah sudah betul atau belum si USER_DO_00 == IO_STATUS_INVALID
		if (RS_SUCC == rs_get_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_00, &status))
		{
			std::cout << "get " << USER_DO_00 << "=" << status << std::endl;
		}
		else
		{
			std::cerr << "get " << USER_DO_00 << " failed" << std::endl;
		}
	}
	else
	{
		std::cerr << "set " << USER_DO_00 << " failed" << std::endl;
	}

}



bool pin_DI_00(RSHD rshd)
{
	double status = 0;
	if (RS_SUCC == rs_get_board_io_status_by_name(rshd, RobotBoardUserDI, USER_DI_00, &status))
	{
		return status;
	}
	else
	{
		return false;
	}
}

bool pin_DI_01(RSHD rshd)
{
	double status = 0;
	if (RS_SUCC == rs_get_board_io_status_by_name(rshd, RobotBoardUserDI, USER_DI_01, &status))
	{
		return status;
	}
	else
	{
		return false;
	}
}

bool pin_DI_02(RSHD rshd)
{
	double status = 0;
	if (RS_SUCC == rs_get_board_io_status_by_name(rshd, RobotBoardUserDI, USER_DI_02, &status))
	{
		return status;
	}
	else
	{
		return false;
	}
}

bool pin_DI_03(RSHD rshd)
{
	double status = 0;
	if (RS_SUCC == rs_get_board_io_status_by_name(rshd, RobotBoardUserDI, USER_DI_03, &status))
	{
		return status;
	}
	else
	{
		return false;
	}
}

bool pin_DI_04(RSHD rshd)
{
	double status = 0;
	if (RS_SUCC == rs_get_board_io_status_by_name(rshd, RobotBoardUserDI, USER_DI_04, &status))
	{
		return status;
	}
	else
	{
		return false;
	}
}

bool pin_DI_05(RSHD rshd)
{
	double status = 0;
	if (RS_SUCC == rs_get_board_io_status_by_name(rshd, RobotBoardUserDI, USER_DI_05, &status))
	{
		return status;
	}
	else
	{
		return false;
	}
}

bool pin_DI_06(RSHD rshd)
{
	double status = 0;
	if (RS_SUCC == rs_get_board_io_status_by_name(rshd, RobotBoardUserDI, USER_DI_06, &status))
	{
		return status;
	}
	else
	{
		return false;
	}
}

bool pin_DI_07(RSHD rshd)
{
	double status = 0;
	if (RS_SUCC == rs_get_board_io_status_by_name(rshd, RobotBoardUserDI, USER_DI_07, &status))
	{
		return status;
	}
	else
	{
		return false;
	}
}

bool pin_DI_17(RSHD rshd)
{
	double status = 0;
	if (RS_SUCC == rs_get_board_io_status_by_name(rshd, RobotBoardUserDI, USER_DI_17, &status))
	{
		return status;
	}
	else
	{
		return false;
	}
}

void pin_D0_00_High(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_00, IO_STATUS_VALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << "D0_00: " << USER_DO_00 << "1" << " ";
	}
	else
	{
		std::cerr << "D0_00: " << USER_DO_00 << "failed" << " ";
	}
}

void pin_D0_01_High(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_01, IO_STATUS_VALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << "D0_01: " << USER_DO_00 << "1" << " ";
	}
	else
	{
		std::cerr << "D0_01: " << USER_DO_00 << "failed" << " ";
	}
}

void pin_D0_02_High(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_02, IO_STATUS_VALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << "D0_02: " << USER_DO_02 << "1" << " ";
	}
	else
	{
		std::cerr << "D0_02: " << USER_DO_02 << "failed" << " ";
	}
}

void pin_D0_03_High(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_03, IO_STATUS_VALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << "D0_03: " << USER_DO_03 << "1" << " ";
	}
	else
	{
		std::cerr << "D0_03: " << USER_DO_03 << "failed" << " ";
	}
}

void pin_D0_04_High(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_04, IO_STATUS_VALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << "D0_04: " << USER_DO_04 << "1" << " ";
	}
	else
	{
		std::cerr << "D0_04: " << USER_DO_04 << "failed" << " ";
	}
}

void pin_D0_05_High(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_05, IO_STATUS_VALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << "D0_05: " << USER_DO_05 << "1" << " ";
	}
	else
	{
		std::cerr << "D0_05: " << USER_DO_05 << "failed" << " ";
	}
}

void pin_D0_06_High(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_06, IO_STATUS_VALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << "D0_06: " << USER_DO_06 << "1" << " ";
	}
	else
	{
		std::cerr << "D0_06: " << USER_DO_06 << "failed" << " ";
	}
}

void pin_D0_07_High(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_07, IO_STATUS_VALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << "D0_07: " << USER_DO_07 << "1" << " ";
	}
	else
	{
		std::cerr << "D0_07: " << USER_DO_07 << "failed" << " ";
	}
}


// ini yg Low
void pin_D0_00_Low(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_00, IO_STATUS_INVALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << "D0_00: " << USER_DO_00 << "0" << " ";
	}
	else
	{
		std::cerr << "D0_00: " << USER_DO_00 << "failed" << " ";
	}
}

void pin_D0_01_Low(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_01, IO_STATUS_INVALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << "D0_01: " << USER_DO_00 << "0" << " ";
	}
	else
	{
		std::cerr << "D0_01: " << USER_DO_00 << "failed" << " ";
	}
}

void pin_D0_02_Low(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_02, IO_STATUS_INVALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << "D0_02: " << USER_DO_02 << "0" << " ";
	}
	else
	{
		std::cerr << "D0_02: " << USER_DO_02 << "failed" << " ";
	}
}

void pin_D0_03_Low(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_03, IO_STATUS_INVALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << "D0_03: " << USER_DO_03 << "0" << " ";
	}
	else
	{
		std::cerr << "D0_03: " << USER_DO_03 << "failed" << " ";
	}
}

void pin_D0_04_Low(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_04, IO_STATUS_INVALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << "D0_04: " << USER_DO_04 << "0" << " ";
	}
	else
	{
		std::cerr << "D0_04: " << USER_DO_04 << "failed" << " ";
	}
}

void pin_D0_05_Low(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_05, IO_STATUS_INVALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << "D0_05: " << USER_DO_05 << "0" << " ";
	}
	else
	{
		std::cerr << "D0_05: " << USER_DO_05 << "failed" << " ";
	}
}

void pin_D0_06_Low(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_06, IO_STATUS_INVALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << "D0_06: " << USER_DO_06 << "0" << " ";
	}
	else
	{
		std::cerr << "D0_06: " << USER_DO_06 << "failed" << " ";
	}
}

void pin_D0_07_Low(RSHD rshd)
{
	double status = 0;

	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_07, IO_STATUS_INVALID))		// IO_STATUS_INVALID -> 0, IO_STATUS_VALID -> 1
	{
		std::cout << "D0_07: " << USER_DO_07 << "0" << " ";
	}
	else
	{
		std::cerr << "D0_07: " << USER_DO_07 << "failed" << " ";
	}
}
// ini yg Low


//机械臂工具端IO测试(必须连接真实机械臂）
void example_ToolIO(RSHD rshd)
{
	double status = 0;

	//首先设置tool_io_0为数字输出
	if (RS_SUCC == rs_set_tool_io_type(rshd, TOOL_DIGITAL_IO_0, IO_OUT))
	{
		//设置tool_io_0数字输出为有效
		if (RS_SUCC == rs_set_tool_do_status(rshd, TOOL_IO_0, IO_STATUS_VALID))
		{
			std::cout<<"set "<<TOOL_IO_0<<" succ"<<std::endl;
		}
		else
		{
			std::cerr<<"set "<<TOOL_IO_0<<" failed"<<std::endl;
		}
		
		//获取tool_io_0数字输出的状态
		if (RS_SUCC == rs_get_tool_io_status(rshd, TOOL_IO_0, &status))
		{
			std::cout<<"get "<<TOOL_IO_0<<"="<<status<<std::endl;
		}
		else
		{
			std::cerr<<"get "<<TOOL_IO_0<<" failed"<<std::endl;
		}
	}
}

/********************************************************************
	function:	example_callbackRobotRoadPoint
	purpose :	实时路点信息回调函数测试
	param   :	rshd 上下文句柄
					
	return  :	true 成功 false 失败
*********************************************************************/
bool example_callbackRobotRoadPoint(RSHD rshd)
{
	bool result = false;

	//允许实时路点信息推送
	if (RS_SUCC == rs_enable_push_realtime_roadpoint(rshd, true))
	{
		if (RS_SUCC == rs_setcallback_realtime_roadpoint(rshd, callback_RealTimeRoadPoint, NULL))
		{
			result = true;
		}
		else
		{
			std::cerr<<"call rs_setcallback_realtime_roadpoint failed"<<std::endl;
		}
	}
	else
		std::cerr<<"call rs_enable_push_realtime_roadpoint failed!"<<std::endl;

	return result;
}

/********************************************************************
function:	Com1_Init
purpose :	使用windows的API函数实现串口操作
param   :	comport 串口号

return  :	true 成功 false 失败
*********************************************************************/
int Com1_Init(int comport,DWORD baud_rate)
{
	DCB dcb;
	COMMTIMEOUTS to;
	//comport = 3;
	DWORD dwError;
	int nPort = 0;
	int ComPort_Array[255];
	int ComPort_Array_Num = 0;
	DWORD BaudRate_n = baud_rate;//bps
	wchar_t szPort[15];
	int i = 0;
	for (i = 0; i<255; i++)
	{
		nPort++;
		if (nPort > 9)
		{
			wsprintf(szPort, L"\\\\.\\COM%d", nPort);
		}
		else
		{
			wsprintf(szPort, L"COM%d", nPort);
		}
		hCom1 = CreateFile(szPort, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);
		if (hCom1 == INVALID_HANDLE_VALUE)
		{
			;
		}
		else
		{
			ComPort_Array[ComPort_Array_Num] = nPort;
			ComPort_Array_Num++;
		}
		CloseHandle(hCom1);
	}
	printf("Useful COM are following:\n");
	for (i = 0; i<ComPort_Array_Num; i++)
	{
		printf("COM%d   ", ComPort_Array[i]);
		if (comport == ComPort_Array[i])
		{
			nPort = comport;
		}
	}
	printf("....\n");
	if (nPort != comport)
	{
		printf("com%d can not be used......\n", comport);
		return 0;
	}
	while (1)
	{
		/////printf("\nPlease input COM:");//by ma
		/////scanf("%d",&nPort);


		if (nPort > 9)
		{
			wsprintf(szPort, L"\\\\.\\COM%d", nPort);
		}
		else
		{
			wsprintf(szPort, L"COM%d", nPort);
		}
		hCom1 = CreateFile(szPort, 
			            GENERIC_READ | GENERIC_WRITE,//允许读和写操作
			            0,//独占方式 
			            NULL, 
						OPEN_EXISTING,//打开存在的串口,必须是OPEN_EXISTING,文件还可以CREATE_NEW,串口不能创建 
						FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, //异步方式打开
						NULL);
		if (hCom1 == INVALID_HANDLE_VALUE)
		{
			dwError = GetLastError();
			printf("Open COM%d error(dwError = %d)...........\n", nPort, dwError);
			return 0;
		}
		else
		{
			SetupComm(hCom1, 1024, 1024);

			to.ReadIntervalTimeout = 0xFFFFFFFF;
			to.ReadTotalTimeoutMultiplier = 0;
			to.ReadTotalTimeoutConstant = 1000;
			to.WriteTotalTimeoutMultiplier = 2;
			to.WriteTotalTimeoutConstant = 5000;
			SetCommTimeouts(hCom1, &to);

			GetCommState(hCom1, &dcb);
			dcb.BaudRate = BaudRate_n;//19200; //波特率为9600
			dcb.ByteSize = 8; //数据位数为7位
			dcb.Parity = NOPARITY;//EVENPARITY; //偶校验
			dcb.StopBits = ONESTOPBIT; //1个停止位
			if (!SetCommState(hCom1, &dcb))
			{
				dwError = GetLastError();
				printf("Set Com State error(dwError = %d)\n", dwError);
				CloseHandle(hCom1);
				return 0;
			}
			else
			{
				printf("COM%d Baud Rate is:%d    \n", nPort, BaudRate_n);
				break;
			}
		}
	}
	SetCommMask(hCom1, EV_RXCHAR);
	m_osWrite1.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	PurgeComm(hCom1, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
	return 1;
}
/********************************************************************
function:	Com2_Init
purpose :	使用windows的API函数实现串口操作
param   :	comport 串口号

return  :	true 成功 false 失败
*********************************************************************/
int Com2_Init(int comport,DWORD baud_rate)
{
	DCB dcb;
	COMMTIMEOUTS to;

	DWORD dwError;
	int nPort = 0;
	int ComPort_Array[255];
	int ComPort_Array_Num = 0;
	DWORD BaudRate_n = baud_rate;//bps
	wchar_t szPort[15];
	int i = 0;
	for (i = 0; i<255; i++)
	{
		nPort++;
		if (nPort > 9)
		{
			wsprintf(szPort, L"\\\\.\\COM%d", nPort);
		}
		else
		{
			wsprintf(szPort, L"COM%d", nPort);
		}
		hCom2 = CreateFile(szPort, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);
		if (hCom2 == INVALID_HANDLE_VALUE)
		{
			;
		}
		else
		{
			ComPort_Array[ComPort_Array_Num] = nPort;
			ComPort_Array_Num++;
		}
		CloseHandle(hCom2);
	}
	printf("Useful COM are following:\n");
	for (i = 0; i<ComPort_Array_Num; i++)
	{
		printf("COM%d   ", ComPort_Array[i]);
		if (comport == ComPort_Array[i])
		{
			nPort = comport;
		}
	}
	printf("....\n");
	if (nPort != comport)
	{
		printf("com%d can not be used......\n", comport);
		return 0;
	}
	while (1)
	{
		/////printf("\nPlease input COM:");//by ma
		/////scanf("%d",&nPort);


		if (nPort > 9)
		{
			wsprintf(szPort, L"\\\\.\\COM%d", nPort);
		}
		else
		{
			wsprintf(szPort, L"COM%d", nPort);
		}
		hCom2 = CreateFile(szPort, 
			            GENERIC_READ | GENERIC_WRITE,//允许读和写操作
			            0,//独占方式 
			            NULL, 
						OPEN_EXISTING,//打开存在的串口,必须是OPEN_EXISTING,文件还可以CREATE_NEW,串口不能创建 
						FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, //异步方式打开
						NULL);
		if (hCom2 == INVALID_HANDLE_VALUE)
		{
			dwError = GetLastError();
			printf("Open COM%d error(dwError = %d)...........\n", nPort, dwError);
			return 0;
		}
		else
		{
			SetupComm(hCom2, 1024, 1024);

			to.ReadIntervalTimeout = 0xFFFFFFFF;
			to.ReadTotalTimeoutMultiplier = 0;
			to.ReadTotalTimeoutConstant = 1000;
			to.WriteTotalTimeoutMultiplier = 2;
			to.WriteTotalTimeoutConstant = 5000;
			SetCommTimeouts(hCom2, &to);

			GetCommState(hCom2, &dcb);
			dcb.BaudRate = BaudRate_n;//19200; //波特率为9600
			dcb.ByteSize = 8; //数据位数为7位
			dcb.Parity = NOPARITY;//EVENPARITY; //偶校验
			dcb.StopBits = ONESTOPBIT; //1个停止位
			if (!SetCommState(hCom2, &dcb))
			{
				dwError = GetLastError();
				printf("Set Com State error(dwError = %d)\n", dwError);
				CloseHandle(hCom2);
				return 0;
			}
			else
			{
				printf("COM%d Baud Rate is:%d    \n", nPort, BaudRate_n);
				break;
			}
		}
	}
	SetCommMask(hCom2, EV_RXCHAR);
	m_osWrite2.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	PurgeComm(hCom2, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
	return 1;
}
/********************************************************************
function:	Com_Hands_SpeedSet
purpose :	使用windows的API函数实现串口操作
param   :	机械手角度值 0-1000

return  :	true 成功 false 失败
*********************************************************************/
int Com_Hands_SpeedSet(int data1, int data2, int data3, int data4, int data5, int data6)
{
	int st = 0;
	short int temp_seekpos_array[6] = { 0 };
	int i = 0;
	CMD1_MC_Array[0] = FRAME_HEAD1;
	CMD1_MC_Array[1] = FRAME_HEAD2;
	CMD1_MC_Array[2] = Hands_ID;
	CMD1_MC_Array[3] = 13;
	CMD1_MC_Array[4] = CMD_MC_SET_DRVALL_SPEED;
	//--------------------------------------------------------------
	temp_seekpos_array[0] = data1;
	temp_seekpos_array[1] = data2;
	temp_seekpos_array[2] = data3;
	temp_seekpos_array[3] = data4;
	temp_seekpos_array[4] = data5;
	temp_seekpos_array[5] = data6;
	for (i = 0; i<6; i++)
	{
		CMD1_MC_Array[i * 2 + 5] = (temp_seekpos_array[i] & 0xFF);
		CMD1_MC_Array[i * 2 + 6] = ((temp_seekpos_array[i] >> 8) & 0xFF);
	}
	st = Uart1_Send_Array((CMD1_MC_Array[3] + 5));
	return st;
}
/********************************************************************
function:	Com_Hands_AngleSet
purpose :	使用windows的API函数实现串口操作
param   :	机械手角度值 0-1000

return  :	true 成功 false 失败
*********************************************************************/
int Com_Hands_AngleSet(int data1, int data2, int data3, int data4, int data5, int data6)
{
	int st = 0;
	short int temp_seekpos_array[6] = { 0 };
	int i = 0;
	CMD1_MC_Array[0] = FRAME_HEAD1;
	CMD1_MC_Array[1] = FRAME_HEAD2;
	CMD1_MC_Array[2] = Hands_ID;
	CMD1_MC_Array[3] = 13;
	CMD1_MC_Array[4] = CMD_MC_SET_DRVALL_SEEKANGLE_GYH;
	//--------------------------------------------------------------
	temp_seekpos_array[0] = data1;
	temp_seekpos_array[1] = data2;
	temp_seekpos_array[2] = data3;
	temp_seekpos_array[3] = data4;
	temp_seekpos_array[4] = data5;
	temp_seekpos_array[5] = data6;
	for (i = 0; i<6; i++)
	{
		CMD1_MC_Array[i * 2 + 5] = (temp_seekpos_array[i] & 0xFF);
		CMD1_MC_Array[i * 2 + 6] = ((temp_seekpos_array[i] >> 8) & 0xFF);
	}
	st = Uart1_Send_Array((CMD1_MC_Array[3] + 5));
	return st;
}
/********************************************************************
function:	Com_Hands_LiKongSet
purpose :	使用windows的API函数实现串口操作
param   :	机械手力控设置

return  :	true 成功 false 失败
*********************************************************************/
int Com_Hands_LiKongSet(int data1, int data2, int data3, int data4, int data5)
{
	int st = 0;
	short int temp_seekpos_array[6] = { 0 };
	int i = 0;
	CMD1_MC_Array[0] = FRAME_HEAD1;
	CMD1_MC_Array[1] = FRAME_HEAD2;
	CMD1_MC_Array[2] = Hands_ID;
	CMD1_MC_Array[3] = 11;
	CMD1_MC_Array[4] = CMD_MC_SET_DRVALL_YBP_THRESHOLD;
	//--------------------------------------------------------------
	temp_seekpos_array[0] = data1;
	temp_seekpos_array[1] = data2;
	temp_seekpos_array[2] = data3;
	temp_seekpos_array[3] = data4;
	temp_seekpos_array[4] = data5;
	for (i = 0; i<5; i++)
	{
		CMD1_MC_Array[i * 2 + 5] = (temp_seekpos_array[i] & 0xFF);
		CMD1_MC_Array[i * 2 + 6] = ((temp_seekpos_array[i] >> 8) & 0xFF);
	}
	st = Uart1_Send_Array((CMD1_MC_Array[3] + 5));
	return st;
}
/********************************************************************
function:	Uart_Send_Array
purpose :	使用windows的API函数实现串口操作
param   :	串口发送指令

return  :	true 成功 false 失败
*********************************************************************/
int Uart1_Send_Array(unsigned int Len)
{
	int st = 0;
	unsigned int i = 0;
	unsigned int check_sum = 0;
	Send1_Len = Len;
	for (i = 2; i<(Send1_Len - 1); i++)
	{
		check_sum = check_sum + CMD1_MC_Array[i];
	}
	CMD1_MC_Array[Send1_Len - 1] = check_sum & 0xFF;

	OVERLAPPED m_osWrite;
	memset(&m_osWrite, 0, sizeof(OVERLAPPED));
	m_osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	DWORD dwBytesWrite = Send1_Len;
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;
	ClearCommError(hCom1, &dwErrorFlags, &ComStat);
	bWriteStat = WriteFile(hCom1, CMD1_MC_Array,
		dwBytesWrite, &dwBytesWrite, &m_osWrite);

	if (!bWriteStat)
	{
		if (GetLastError() == ERROR_IO_PENDING)
		{
			WaitForSingleObject(m_osWrite.hEvent, 1000);
		}
	}
	Sleep(300);
	st = Uart1_Rec();
	return st;
}

/********************************************************************
function:	Uart_Rec
purpose :	使用windows的API函数实现串口操作
param   :	串口接收指令

return  :	true 成功 false 失败
*********************************************************************/
int Uart1_Rec(void)
{
	OVERLAPPED m_osRead;
	memset(&m_osRead, 0, sizeof(OVERLAPPED));
	m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

	COMSTAT ComStat;
	DWORD dwErrorFlags;

	memset(Com1_RecBuf, '\0', 100);
	DWORD dwBytesRead = 100;//读取的字节数
	BOOL bReadStat;

	ClearCommError(hCom1, &dwErrorFlags, &ComStat);
	dwBytesRead = min(dwBytesRead, (DWORD)ComStat.cbInQue);
	bReadStat = ReadFile(hCom1, Com1_RecBuf,
		dwBytesRead, &dwBytesRead, &m_osRead);
	if (!bReadStat)
	{
		if (GetLastError() == ERROR_IO_PENDING)
			//GetLastError()函数返回ERROR_IO_PENDING,表明串口正在进行读操作
		{
			WaitForSingleObject(m_osRead.hEvent, 1000);
			//使用WaitForSingleObject函数等待，直到读操作完成或延时已达到2秒钟
			//当串口读操作进行完毕后，m_osRead的hEvent事件会变为有信号
		}
	}
	Rec1_Len = dwBytesRead;
	int st = Uart1_MC_JieXi();
	printf("Cnt=%d,Hands_CMD_Return=%d\r\n", Rec1_Len,st);
	PurgeComm(hCom1, PURGE_TXABORT |
		PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);

	return st;
}
/********************************************************************
function:	Uart_MC_JieXi
purpose :	使用windows的API函数实现串口操作
param   :	串口接收指令解析

return  :	true 成功 false 失败
*********************************************************************/
bool Uart1_MC_JieXi(void)
{
	bool result = false;
	int k = 0;
	int Status_Rec = 0;
	int data = 0;
	int Rx_Len = 0;
	int ki = 0;
	int checksum = 0;
	for (k = 0; k < Rec1_Len; k++)
	{
		data = Com1_RecBuf[k];
		switch (Status_Rec)
		{
			case 0:
					{
						if (data == FRAME_HEAD1_RT)
						{
							Status_Rec = 1;
						}
						break;
					}
			case 1:
					{
						if (data == FRAME_HEAD2_RT)
						{
							Status_Rec = 2;
						}
						else if (data == FRAME_HEAD1_RT)
						{
							Status_Rec = 1;
						}
						else
						{
							Status_Rec = 0;
						}
						break;
					}
			case 2://ID
					{
						if (data == Hands_ID)
						{
							Status_Rec = 3;
						}
						else if (data == FRAME_HEAD1_RT)
						{
							Status_Rec = 1;
						}
						else
						{
							Status_Rec = 0;
						}
						break;
					}
			case 3://lens
					{
						CMD1_MC_REC_Array[0] = FRAME_HEAD1_RT;
						CMD1_MC_REC_Array[1] = FRAME_HEAD2_RT;
						CMD1_MC_REC_Array[2] = Hands_ID;
						CMD1_MC_REC_Array[3] = data;
						Rx_Len = data + 5;
						Status_Rec = 4;
						ki = 4;
						break;
					}
			case 4://others
					{
						if (ki == (Rx_Len-1))
						{
							CMD1_MC_REC_Array[ki] = data;
							checksum = 0;
							for (int t = 2; t < (Rx_Len - 1); t++)
							{
								checksum = checksum + CMD1_MC_REC_Array[t];
							}
							if (checksum == data)
							{
								result = true;
								MC1_Rec_Pro();
							}
						}
						else
						{
							CMD1_MC_REC_Array[ki] = data;
							ki++;
						}
						break;
					}
			default:
					{
						break;
					}
		}
	}
	return result;
}
/********************************************************************
function:	Com_Btn_LED_Set
purpose :	使用windows的API函数实现串口操作
param   :	控制LED灯亮灭，2-亮 1-灭

return  :	true 成功 false 失败
*********************************************************************/
int Com_Btn_LED_Set(int data1)
{
	int st = 0;
	int set_cnt = 0;
	int repeat_cnt = 0;
	short int temp_seekpos_array[6] = { 0 };
	int i = 0,k = 0;
	CMD2_MC_Array[0] = FRAME_HEAD1;
	CMD2_MC_Array[1] = FRAME_HEAD2;
	CMD2_MC_Array[2] = Hands_ID;
	CMD2_MC_Array[3] = 2;
	CMD2_MC_Array[4] = CMD_MC_SET_DJ_TURN;
	//--------------------------------------------------------------
	temp_seekpos_array[0] = data1;
	CMD2_MC_Array[5] = (temp_seekpos_array[i] & 0xFF);
	
	set_cnt = 40;
	repeat_cnt = 4;
	st = 0;
	for (k = 0; k < repeat_cnt; k++)
	{
		st = Uart2_Send_Array((CMD2_MC_Array[3] + 5));
		for (i = 0; i < set_cnt; i++)
		{
			Sleep(50);
			if (1 == Uart2_Rec())
			{
				if (State_Led == data1)
				{
					st = 1;
					i = set_cnt + 1;
					k = repeat_cnt + 1;
				}

			}
		}
	}
	if (st == 0)
	{
		printf("button set error...\n");
		system("Pause");
	}
	return st;
}

/********************************************************************
function:	Uart2_Send_Array
purpose :	使用windows的API函数实现串口操作
param   :	串口发送指令

return  :	true 成功 false 失败
*********************************************************************/
int Uart2_Send_Array(unsigned int Len)
{
	unsigned int i = 0;
	unsigned int check_sum = 0;
	Send2_Len = Len;
	for (i = 2; i<(Send2_Len - 1); i++)
	{
		check_sum = check_sum + CMD2_MC_Array[i];
	}
	CMD2_MC_Array[Send2_Len - 1] = check_sum & 0xFF;

	OVERLAPPED m_osWrite;
	memset(&m_osWrite, 0, sizeof(OVERLAPPED));
	m_osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	DWORD dwBytesWrite = Send2_Len;
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;
	ClearCommError(hCom2, &dwErrorFlags, &ComStat);
	bWriteStat = WriteFile(hCom2, CMD2_MC_Array,
		dwBytesWrite, &dwBytesWrite, &m_osWrite);

	if (!bWriteStat)
	{
		if (GetLastError() == ERROR_IO_PENDING)
		{
			WaitForSingleObject(m_osWrite.hEvent, 1000);
		}
	}
	return 1;
}
/********************************************************************
function:	Uart_Rec
purpose :	使用windows的API函数实现串口操作
param   :	串口接收指令

return  :	true 成功 false 失败
*********************************************************************/
int Uart2_Rec(void)
{
	OVERLAPPED m_osRead2;
	memset(&m_osRead2, 0, sizeof(OVERLAPPED));
	m_osRead2.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

	COMSTAT ComStat;
	DWORD dwErrorFlags;

	memset(Com2_RecBuf, '\0', 100);
	DWORD dwBytesRead = 100;//读取的字节数
	BOOL bReadStat;

	ClearCommError(hCom2, &dwErrorFlags, &ComStat);
	dwBytesRead = min(dwBytesRead, (DWORD)ComStat.cbInQue);
	bReadStat = ReadFile(hCom2, Com2_RecBuf,
		dwBytesRead, &dwBytesRead, &m_osRead2);
	if (!bReadStat)
	{
		if (GetLastError() == ERROR_IO_PENDING)
			//GetLastError()函数返回ERROR_IO_PENDING,表明串口正在进行读操作
		{
			WaitForSingleObject(m_osRead2.hEvent, 1000);
			//使用WaitForSingleObject函数等待，直到读操作完成或延时已达到2秒钟
			//当串口读操作进行完毕后，m_osRead的hEvent事件会变为有信号
		}
	}
	Rec2_Len = dwBytesRead;
	int st = Uart2_MC_JieXi();
	//printf("Cnt=%d,Hands_CMD_Return=%d\r\n", Rec2_Len, st);
	PurgeComm(hCom2, PURGE_TXABORT |
		PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);

	return st;
}
/********************************************************************
function:	Uart2_MC_JieXi
purpose :	使用windows的API函数实现串口操作
param   :	串口接收指令解析

return  :	true 成功 false 失败
*********************************************************************/
bool Uart2_MC_JieXi(void)
{
	bool result = false;
	int k = 0;
	int Status_Rec = 0;
	int data = 0;
	int Rx_Len = 0;
	int ki = 0;
	int checksum = 0;
	for (k = 0; k < Rec2_Len; k++)
	{
		data = Com2_RecBuf[k];
		switch (Status_Rec)
		{
		case 0:
		{
			if (data == FRAME_HEAD1_RT)
			{
				Status_Rec = 1;
			}
			break;
		}
		case 1:
		{
			if (data == FRAME_HEAD2_RT)
			{
				Status_Rec = 2;
			}
			else if (data == FRAME_HEAD1_RT)
			{
				Status_Rec = 1;
			}
			else
			{
				Status_Rec = 0;
			}
			break;
		}
		case 2://ID
		{
			if (data == Hands_ID)
			{
				Status_Rec = 3;
			}
			else if (data == FRAME_HEAD1_RT)
			{
				Status_Rec = 1;
			}
			else
			{
				Status_Rec = 0;
			}
			break;
		}
		case 3://lens
		{
			CMD2_MC_REC_Array[0] = FRAME_HEAD1_RT;
			CMD2_MC_REC_Array[1] = FRAME_HEAD2_RT;
			CMD2_MC_REC_Array[2] = Hands_ID;
			CMD2_MC_REC_Array[3] = data;
			Rx_Len = data + 5;
			Status_Rec = 4;
			ki = 4;
			break;
		}
		case 4://others
		{
			if (ki == (Rx_Len - 1))
			{
				CMD2_MC_REC_Array[ki] = data;
				checksum = 0;
				for (int t = 2; t < (Rx_Len - 1); t++)
				{
					checksum = checksum + CMD2_MC_REC_Array[t];
				}
				if (checksum == data)
				{
					result = true;
					MC2_Rec_Pro();
				}
			}
			else
			{
				CMD2_MC_REC_Array[ki] = data;
				ki++;
			}
			break;
		}
		default:
		{
			break;
		}
		}
	}
	return result;
}
/********************************************************************
function:	MC1_Rec_Pro
purpose :	使用windows的API函数实现串口操作
param   :	串口接收指令处理

return  :	true 成功 false 失败
*********************************************************************/
void MC1_Rec_Pro(void)
{

}
/********************************************************************
function:	MC2_Rec_Pro
purpose :	使用windows的API函数实现串口操作
param   :	串口接收指令处理

return  :	true 成功 false 失败
*********************************************************************/
void MC2_Rec_Pro(void)
{
	if (CMD2_MC_REC_Array[4] == 0x3D)//btn status
	{
		State_Btn = CMD2_MC_REC_Array[5];
		State_Led = CMD2_MC_REC_Array[6];
	}
}