#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <dosCore/wsRobotClient.h>
#include <cstdlib>
#include <sstream>
#include <functional>
#include <unordered_map>
#include <chrono>

namespace DosClient
{

class PluginClient;
#define REGISTER_PLUGINCLIENT_FACTORY(TYPE, IDSTR) static PluginClientFactoryBuild<TYPE> xx_##TYPE(IDSTR);

template<typename CreateClass>
class PluginClientFactoryBuild
{
public:
	PluginClientFactoryBuild(const std::string& idIn)
	{
		auto createFn = []() -> std::unique_ptr<PluginClient>
		{
			return std::make_unique<CreateClass>();
		};

		auto newClient = createFn();
		PluginFactoryMgr::registerFactory(idIn, createFn);
		std::cout << "PluginFactoryMgr::() - Registring Plugin <" << idIn << ">" << std::endl;
	}
};

class PluginFactoryMgr
{
public:
	using FactoryMap = std::unordered_map<std::string, std::function<std::unique_ptr<PluginClient>()>>;

public:
	static FactoryMap& getFactoryMap();

	static void registerFactory(const std::string& typeIn, std::function<std::unique_ptr<PluginClient>()> fnIn);

	static std::shared_ptr<PluginClient> getPluginClient(const std::string& typeIn);
};

class PluginClient
{
public:
	PluginClient(const std::string layerStr="") : mCamLayerStr(layerStr) {}
	virtual ~PluginClient() {}

	bool canConnect() const { return true; }
	virtual void connect(const std::string& address, int port);
	virtual void prepareClient(std::shared_ptr<DosClient::WsRobotClient>& sClientIn) = 0;
	virtual void setupData(nlohmann::json& jsonObj) = 0;
	virtual void tickGUI() {}
	virtual void recvPacket(BaseCommandPacket* pktIn);
	virtual bool canCreateCameras() const { return false; }
	virtual void createCameras() {}
	virtual void streamCamerasFrameStep() {}
	virtual void setStreamCameraQuality(int valIn) { valIn = 0; }
	virtual int getStreamCameraQuality() const { return 0; }

	std::shared_ptr<DosClient::WsRobotClient> sClient;
	// (posX, posY, scaleX, scaleY, name, description, layerStr)
	std::function<void(int, int, int, int, const std::string&, const std::string&)> mCreateCamFn;
	std::string mCamLayerStr;
	std::vector<uint8_t> mCamBytes;
	std::function<void(DosClient::PluginClient *)> mStreamFn;
};

class PluginManager
{
public:
	PluginManager(const std::string& configJsonStr);

	int numClients() const;
	const std::unordered_map<std::string, std::shared_ptr<PluginClient>>& getClients() const;
	std::vector<std::string> getClientTypes() const;
	PluginClient * getClientWithLayerStr(const std::string& layerStr) const;
	PluginClient * getClientWithTypeName(const std::string& nameIn) const;
	void drawGUI();

protected:
	std::unordered_map<std::string, std::shared_ptr<PluginClient>> mPluginClients;
};

} // end of namespace SpotClient