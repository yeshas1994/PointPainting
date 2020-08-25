#pragma once
#include "pluginServices.h"
#include <nlohmann/json.hpp>
#include <mutex>

namespace DosClient
{

/*
A map of Region of Interests keyed by name:
{
	"RIO_NAME_1" :
	{
		"pos" : [0, 0], // [PosX , PosY]
		"size" : [100, 100], // [Width, Height]
		"freq" : [ Array of floats for histogram ],
		"min" : 28.0, // Minimum value of starting range for histogram
		"max" : 36.0 // Maximum value of ending range for histogram
	},
	...
}
*/

class PluginThermalCamClient : public PluginClient
{
public:
	class ThermalData
	{
	public:
		ThermalData() = default;

		std::pair<int, int> getPlotRangeIdx(float tempStart, float tempEnd) const;
		void fillFromJSON(nlohmann::json& jsonIn);

	public:
		int mPosX = 0;
		int mPosY = 0;
		int mWidth = 100;
		int mHeight = 100;
		float mTempMin = 28.0f;
		float mTempMax = 36.0f;
		std::vector<float> mFreqVals;
	};

public:
	PluginThermalCamClient();
	virtual ~PluginThermalCamClient();

	virtual void setupData(nlohmann::json& jsonObj) override;
	virtual void tickGUI() override;
	virtual void prepareClient(std::shared_ptr<DosClient::WsRobotClient>& sClientIn) override;
	virtual bool canCreateCameras() const { return true; }
	virtual void createCameras() override;
	virtual void streamCamerasFrameStep() override;
	virtual void setStreamCameraQuality(int valIn) override;
	virtual int getStreamCameraQuality() const override { return m_camQuality; }

	void fillAllThermalData(nlohmann::json& jsonIn);
	ThermalData* getThermalData(const std::string& roiName);

protected:
	void sendStatusPkt(nlohmann::json& jsonIn);
	
protected:
	int m_drawCnt = 0;
	int m_camQuality = 0;
	std::mutex m_drawLock;
	// Plotting the Histogram with imGUI
	std::unordered_map<std::string, ThermalData> mThermalDataMap;
	std::string mActiveROI;
	int mActiveIdx = 0;
	std::vector<std::string> mROINames;
	float mTempStart = 33.0f;
	float mTempEnd = 45.0f;
	std::mutex mDataLock;
};

} // end of namespace SpotClient