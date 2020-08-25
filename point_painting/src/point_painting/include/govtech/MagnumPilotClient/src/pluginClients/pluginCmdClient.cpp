#include "pluginCmdClient.h"
#include "dosCore/commandPacket.h"
#include "dosCore/commandPacketParse.h"
#include <iomanip>
#include <imguiLib/imgui.h>
#include <algorithm>

#if defined(_WIN32)
#include <windows.h>
#include <direct.h>
#include <stdlib.h>
#include <stdio.h>
#else
#include <dirent.h>
#include <errno.h>
#endif

namespace DosClient
{

PluginCmdClient::PluginCmdClient()
	: PluginClient("CmdCam")
{
}

PluginCmdClient::~PluginCmdClient()
{
}

void PluginCmdClient::setupData(nlohmann::json& jsonObj)
{
}

void PluginCmdClient::tickGUI()
{
	ImGui::BulletText("Remote Commands");
	if (mCmdList.empty())
	{
		return;
	}

	std::array<const char*, 512> cCmdChars;
	for (size_t i = 0; i < mCmdList.size(); i++)
	{
		cCmdChars[i] = mCmdList[i].c_str();
	}

	ImGui::Combo("Commands", &mCmdIdx, cCmdChars.data(), (int)mCmdList.size());
	if (ImGui::Button("Run")) {
		nlohmann::json sJSON;
		sJSON["cmd"] = "run";
		sJSON["runType"] = mCmdList[mCmdIdx];
		sendStatusPkt(sJSON);
	}
}

void PluginCmdClient::prepareClient(std::shared_ptr<DosClient::WsRobotClient>& sClientIn)
{
	// Receive status msg
	sClientIn->registerPacketRecvFn(
		COMMAND_STATUS_TYPE,
		[this](BaseCommandPacket* pktIn) -> void
		{
			if (auto statusPkt = dynamic_cast<StatusCommandPacket*>(pktIn))
			{
				auto jsonObj = statusPkt->getStatusJSON();
				std::lock_guard<std::mutex> scopeLock(mDataLock);
				if (jsonObj.count("cmdList") > 0)
				{
					mCmdList = jsonObj["cmdList"].get<std::vector<std::string>>();
				}
			}
		}
	);

	// Get Command List
	nlohmann::json sJSON;
	sJSON["cmd"] = "list";
	sendStatusPkt(sJSON);
}

void PluginCmdClient::sendStatusPkt(nlohmann::json& jsonIn)
{
	std::unique_ptr<BaseCommandPacket> sendPkt = std::make_unique<StatusCommandPacket>();
	dynamic_cast<StatusCommandPacket*>(sendPkt.get())->getStrMsg() = jsonIn.dump();
	sClient->addMsg(sendPkt);
}


REGISTER_PLUGINCLIENT_FACTORY(PluginCmdClient, "Cmd")

} // end of namespace DosClient