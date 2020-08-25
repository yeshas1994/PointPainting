#include <pluginClients/pluginServices.h>
#include <nlohmann/json.hpp>
#include <imguiLib/imgui.h>

namespace DosClient
{

//PluginClient
void PluginClient::connect(const std::string& address, int port)
{
	if (!canConnect()) { return; }
	sClient = WsClient::startClient<WsRobotClient>("ws://" + address, port);
	sClient->setPacketRecvCB([this](BaseCommandPacket* pktIn) {
		recvPacket(pktIn);
	});
	prepareClient(sClient);
}

void PluginClient::recvPacket(BaseCommandPacket* pktIn)
{
	if (pktIn)
	{

	}
}

// PluginFactoryMgr
PluginFactoryMgr::FactoryMap& PluginFactoryMgr::getFactoryMap()
{
	static FactoryMap classRegister{};
	return classRegister;
}

void PluginFactoryMgr::registerFactory(const std::string& typeIn, std::function<std::unique_ptr<PluginClient>()> fnIn)
{
	if (getFactoryMap().find(typeIn) != getFactoryMap().end())
	{
		std::cout << "PluginFactoryMgr::registerFactory() - ERROR! A type of id: " << typeIn << " already exists!" << std::endl;
	}

	getFactoryMap()[typeIn] = fnIn;
}

std::shared_ptr<PluginClient> PluginFactoryMgr::getPluginClient(const std::string& typeIn)
{
	std::unique_ptr<PluginClient> retClient;
	auto& cFactoryMap = getFactoryMap();
	auto findFactory = cFactoryMap.find(typeIn);
	if (findFactory != cFactoryMap.end())
	{
		auto& createFunc = findFactory->second;
		retClient = createFunc();
	}
	else {
		std::cout << "PluginFactoryMgr::getPluginClient() - ERROR! Invalid factory id: " << typeIn << std::endl;
	}
	return retClient;
}

// PluginManager
PluginManager::PluginManager(const std::string& configJsonStr)
{
	nlohmann::json rJSON = nlohmann::json::parse(configJsonStr); // Ignore Intellisense error, this works
	if (rJSON.count("plugins") == 0)
	{
		return;
	}

	auto& pluginObjs = rJSON["plugins"];
	for (nlohmann::json::iterator it = pluginObjs.begin(); it != pluginObjs.end(); ++it)
	{
		const std::string& pluginType = it.key();
		auto& pluginObj = it.value();
		if ((pluginObj.count("address") > 0) && (pluginObj.count("port") > 0))
		{
			std::string cAddress = pluginObj["address"].get<std::string>();
			int cPort = pluginObj["port"].get<int>();
			bool cActive = true;
			if (pluginObj.count("active") > 0)
			{
				cActive = pluginObj["active"].get<bool>();
				if (cActive) {
					std::cout << "The Plugin: " << pluginType << " is ACTIVE" << std::endl;
				}
			}

			if (cActive) {
				auto newPlugin = PluginFactoryMgr::getPluginClient(pluginType);
				if (newPlugin) {
					newPlugin->connect(cAddress, cPort);

					if (pluginObj.count("data") > 0)
					{
						newPlugin->setupData(pluginObj["data"]);
					}

					mPluginClients[pluginType] = std::move(newPlugin);
				}
			}
		}
	}
}

int PluginManager::numClients() const
{
	return static_cast<int>(mPluginClients.size());
}

const std::unordered_map<std::string, std::shared_ptr<PluginClient>>& PluginManager::getClients() const
{
	return mPluginClients;
}

std::vector<std::string> PluginManager::getClientTypes() const
{
	std::vector<std::string> retList;
	for (const auto& cPair : mPluginClients)
	{
		retList.push_back(cPair.first);
	}
	return retList;
}

PluginClient * PluginManager::getClientWithLayerStr(const std::string& layerStr) const
{
	for (const auto& cPair : mPluginClients)
	{
		auto& cPlugin = cPair.second;
		if(cPlugin->mCamLayerStr == layerStr)
		{
			return cPlugin.get();
		}
	}
	return nullptr;
}

PluginClient * PluginManager::getClientWithTypeName(const std::string& nameIn) const
{
	for (const auto& cPair : mPluginClients)
	{
		auto& cName = cPair.first;
		if(cName == nameIn)
		{
			auto& cPlugin = cPair.second;
			return cPlugin.get();
		}
	}
	return nullptr;
}

void PluginManager::drawGUI()
{
	if (mPluginClients.empty())
	{
		return;
	}

	ImGui::Begin("LIMA Plugins");
	for (auto& cPair : mPluginClients)
	{
		auto& cName = cPair.first;
		auto& cClient = cPair.second;
		if (ImGui::CollapsingHeader(cName.c_str()))
		{
			cClient->tickGUI();
		}
	}
	ImGui::End();
}

} // end of namespace SpotClient