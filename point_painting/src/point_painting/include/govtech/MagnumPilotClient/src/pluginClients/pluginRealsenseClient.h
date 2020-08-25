#pragma once
#include "pluginServices.h"
#include <mutex>

namespace DosClient
{

class PluginRealsenseClient : public PluginClient
{
public:
	PluginRealsenseClient();
	virtual ~PluginRealsenseClient();

	virtual void setupData(nlohmann::json& jsonObj) override;
	virtual void tickGUI() override;
	virtual void prepareClient(std::shared_ptr<DosClient::WsRobotClient>& sClientIn) override;
	virtual bool canCreateCameras() const { return true; }
	virtual void createCameras() override;
	virtual void streamCamerasFrameStep() override;
	virtual void setStreamCameraQuality(int valIn) override;
	virtual int getStreamCameraQuality() const override { return m_camQuality; }

	bool getInfraRed() const;
	void enableInfraRed(bool flagIn);

	std::string getResourcePath(const std::string& pathIn, const std::string& resourceType) const;

protected:
	void clearRunFrameStrList();
	void addRunFrameStrList(const std::string& valIn);
	void beginRetrieveFrames(const std::string& pathIn);
	void requestFrameData(int idx);
	void sendStatusPkt(nlohmann::json& jsonIn);
	
protected:
	std::string m_outputFolder = "./RunOutput";
	int m_drawCnt = 0;
	nlohmann::json m_recordInfo;
	std::vector<char*> m_runIdxStrList;
	std::vector<char*> m_runFramesStrList;
	int m_runIdx = 0;
	bool m_isRecording = false;
	int m_retrieveFrameIdx = 0;
	int m_retrieveFrameNum = 0;
	bool m_isRetrieving = false;
	int m_camQuality = 0;
	bool m_infraRed = false;
	std::mutex m_drawLock;
};

} // end of namespace SpotClient