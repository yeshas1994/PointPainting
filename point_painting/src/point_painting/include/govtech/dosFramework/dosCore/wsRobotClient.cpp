#include <dosCore/wsRobotClient.h>
#include <dosCore/commandPacketParse.h>
#include <msgpack11.hpp>
#include <stb_image.h>

#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <functional>

namespace DosClient
{
	WsRobotClient::WsRobotClient(): WsClient()
	{}

	void WsRobotClient::powerRobot(bool flagIn)
	{
		std::unique_ptr<BaseCommandPacket> newPkt = std::make_unique<PowerRobotCommandPacket>();
		dynamic_cast<PowerRobotCommandPacket*>(newPkt.get())->setPower(flagIn);
		addMsg(newPkt);
	}

	void WsRobotClient::commandRobot(RobotCommandPacket::ROBOT_CMD cmdIn)
	{
		std::unique_ptr<BaseCommandPacket> newPkt = std::make_unique<RobotCommandPacket>();
		dynamic_cast<RobotCommandPacket*>(newPkt.get())->setRobotCommand(cmdIn);
		addMsg(newPkt);
	}

	void WsRobotClient::moveVelRobot(MoveVelCommandPacket::VEL_CMD cmdIn, float magnitude)
	{
		std::unique_ptr<BaseCommandPacket> newPkt = std::make_unique<MoveVelCommandPacket>();
		dynamic_cast<MoveVelCommandPacket*>(newPkt.get())->setVelCmd(cmdIn, magnitude);
		addMsg(newPkt);
	}

	void WsRobotClient::freeMoveVelRobot(float xMag, float yMag, float turnMag, float maxMagnitude)
	{
		std::unique_ptr<BaseCommandPacket> newPkt = std::make_unique<MoveVelCommandPacket>();
		dynamic_cast<MoveVelCommandPacket*>(newPkt.get())->setVelFreeMove(xMag, yMag, turnMag, maxMagnitude);
		addMsg(newPkt);
	}

	void WsRobotClient::sendWaypoints(const float* pFloats, unsigned num)
	{
		std::unique_ptr<BaseCommandPacket> newPkt = std::make_unique<SetWaypointsPacket>();
		dynamic_cast<SetWaypointsPacket*>(newPkt.get())->setWaypointsVector(pFloats, num);
		addMsg(newPkt);
	}

	void WsRobotClient::moveToWaypoint(unsigned index)
	{
		std::unique_ptr<BaseCommandPacket> newPkt = std::make_unique<MoveToWaypointPacket>();
		dynamic_cast<MoveToWaypointPacket*>(newPkt.get())->setWaypointIndex(index);
		addMsg(newPkt);
	}

	void WsRobotClient::clearWaypoints()
	{
		sendRobotMsgStr("clearWaypoints");
	}

	void WsRobotClient::updatePose(float pX, float pY, float pZ, float qX, float qY, float qZ, float qW)
	{
		std::unique_ptr<BaseCommandPacket> newPkt = std::make_unique<UpdatePosePacket>();
		dynamic_cast<UpdatePosePacket*>(newPkt.get())->set(pX, pY, pZ, qX, qY, qZ, qW);
		addMsg(newPkt);
	}

	void WsRobotClient::initializePosition(float pX, float pY, float angle)
	{
		std::unique_ptr<BaseCommandPacket> newPkt = std::make_unique<InitialPositionPacket>();
		dynamic_cast<InitialPositionPacket*>(newPkt.get())->set(pX, pY, angle);
		addMsg(newPkt);
	}

	void WsRobotClient::sendRobotMsgStr(const std::string& msgIn)
	{
		std::unique_ptr<BaseCommandPacket> newPkt = std::make_unique<MsgStrCommandPacket>();
		dynamic_cast<MsgStrCommandPacket*>(newPkt.get())->getStrMsg() = msgIn;
		addMsg(newPkt);
	}

	void WsRobotClient::sendRobotStatusStr()
	{
		sendRobotMsgStr("status");
	}

	void WsRobotClient::sendRobotOdomStr()
	{
		sendRobotMsgStr("odom");
	}

	void WsRobotClient::sendRobotControlStateStartRecord()
	{
		sendRobotMsgStr("controlStateStartRecord");
	}

	void WsRobotClient::sendRobotControlStateEndRecord()
	{
		sendRobotMsgStr("controlStateEndRecord");
	}

	void WsRobotClient::takeRobot()
	{
		sendRobotMsgStr("take");
	}

	void WsRobotClient::sendRobotControlPermissions(int priority, bool canMove, bool alwaysReceive)
	{
		std::unique_ptr<BaseCommandPacket> newPkt = std::make_unique<ControlPermissionsPacket>();
		auto permPkt = dynamic_cast<ControlPermissionsPacket*>(newPkt.get());
		nlohmann::json setJSON;
		setJSON["priority"] = priority;
		setJSON["canMove"] = canMove;
		setJSON["alwaysReceive"] = alwaysReceive;
		permPkt->getStrMsg() = setJSON.dump();
		addMsg(newPkt);
	}

	void WsRobotClient::sendImageStreamToggle(bool flagIn, const std::vector<std::string>& imageNames)
	{
		std::unique_ptr<BaseCommandPacket> newPkt = std::make_unique<ImageStreamTogglePacket>();
		auto imgTogglePkt = dynamic_cast<ImageStreamTogglePacket*>(newPkt.get());
		imgTogglePkt->setStream(flagIn);
		imgTogglePkt->setImageNames(imageNames);
		addMsg(newPkt);
	}

	void WsRobotClient::sendImage(const std::vector<ImageListCommandPacket::SubImage>& imageIn) {
		std::unique_ptr<BaseCommandPacket> newPkt = std::make_unique<ImageListCommandPacket>();
		dynamic_cast<ImageListCommandPacket*>(newPkt.get())->setImage(imageIn);
		addMsg(newPkt); 
	}

	void WsRobotClient::videoRecord(bool start)
	{
		sendRobotMsgStr(start ? "VIDEO_RECORD_START" : "VIDEO_RECORD_STOP");
	}

	void WsRobotClient::setRobotMotionParams(const nlohmann::json& mParams)
	{
		std::unique_ptr<BaseCommandPacket> newPkt = std::make_unique<MotionParamsCommandPacket>();
		auto motionParamsPkt = dynamic_cast<MotionParamsCommandPacket*>(newPkt.get());
		motionParamsPkt->getStrMsg() = mParams.dump();
		addMsg(newPkt);
	}

	void WsRobotClient::setRobotSettings(const nlohmann::json& mParams)
	{
		std::unique_ptr<BaseCommandPacket> newPkt = std::make_unique<SettingsCommandPacket>();
		auto settingsPkt = dynamic_cast<SettingsCommandPacket*>(newPkt.get());
		settingsPkt->getStrMsg() = mParams.dump();
		addMsg(newPkt);
	}

	void WsRobotClient::sendRobotSpotGraphNav(const nlohmann::json& mParams)
	{
		std::unique_ptr<BaseCommandPacket> newPkt = std::make_unique<SpotGraphNavCommandPacket>();
		auto gNavPkt = dynamic_cast<SpotGraphNavCommandPacket*>(newPkt.get());
		gNavPkt->getStrMsg() = mParams.dump();
		addMsg(newPkt);
	}
}