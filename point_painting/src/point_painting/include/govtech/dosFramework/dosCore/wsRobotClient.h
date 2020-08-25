#pragma once
#include <dosCore/wsClient.h>

namespace DosClient
{
	class WsRobotClient : public WsClient
	{
	public:
		WsRobotClient();

		void powerRobot(bool flagIn);
		void commandRobot(RobotCommandPacket::ROBOT_CMD cmdIn);
		void moveVelRobot(MoveVelCommandPacket::VEL_CMD cmdIn, float magnitude);
		void freeMoveVelRobot(float xMag, float yMag, float turnMag, float maxMagnitude);
		void sendWaypoints(const float *pFloats, unsigned num);
		void moveToWaypoint(unsigned index);
		void clearWaypoints();
		void updatePose(float pX, float pY, float pZ, float qX, float qY, float qZ, float qW);
		void initializePosition(float pX, float pY, float angle);
		void sendRobotMsgStr(const std::string& msgIn);
		void sendRobotStatusStr();
		void sendRobotOdomStr();
		void sendRobotControlStateStartRecord();
		void sendRobotControlStateEndRecord();
		void takeRobot();
		void sendRobotControlPermissions(int priority, bool canMove, bool alwaysReceive);
		void sendImageStreamToggle(bool flagIn, const std::vector<std::string>& imageNames);
		void sendImage(const std::vector<ImageListCommandPacket::SubImage>& imageIn);
		void videoRecord(bool start);
		void setRobotMotionParams(const nlohmann::json& mParams);
		void setRobotSettings(const nlohmann::json& mParams);
		void sendRobotSpotGraphNav(const nlohmann::json& mParams);

	};
}