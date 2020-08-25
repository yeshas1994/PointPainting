#pragma once
#include "pluginServices.h"
#include <nlohmann/json.hpp>
#include <mutex>

namespace DosClient
{

class PluginCmdClient : public PluginClient
{
public:
	PluginCmdClient();
	virtual ~PluginCmdClient();

	virtual void setupData(nlohmann::json& jsonObj) override;
	virtual void tickGUI() override;
	virtual void prepareClient(std::shared_ptr<DosClient::WsRobotClient>& sClientIn) override;
	virtual bool canCreateCameras() const { return false; }

protected:
	void sendStatusPkt(nlohmann::json& jsonIn);

protected:
	std::mutex mDataLock;
	std::vector<std::string> mCmdList;
	int mCmdIdx = 0;
};

} // end of namespace DosClient