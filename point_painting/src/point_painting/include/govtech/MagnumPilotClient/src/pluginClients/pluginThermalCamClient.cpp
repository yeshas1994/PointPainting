#include "pluginThermalCamClient.h"
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
// ThermalData
std::pair<int, int> PluginThermalCamClient::ThermalData::getPlotRangeIdx(float tempStart, float tempEnd) const
{
	auto inValRange = [](float valIn, float aStart, float bStart) -> bool
	{
		return (valIn >= aStart) && (valIn <= bStart);
	};

	int retStart = 0;
	int retEnd = 0;

	float step = (mTempMax - mTempMin) / static_cast<float>(mFreqVals.size());
	float cTemp = mTempMin;
	for (size_t i = 0; i < mFreqVals.size(); i++)
	{
		float aTemp = cTemp;
		float bTemp = cTemp + step;
		if (inValRange(tempStart, aTemp, bTemp))
		{
			retStart = static_cast<int>(i);
		}

		if (inValRange(tempEnd, aTemp, bTemp))
		{
			retEnd = static_cast<int>(i);
		}
		cTemp += step;
	}
	return std::make_pair(retStart, retEnd);
}

void PluginThermalCamClient::ThermalData::fillFromJSON(nlohmann::json& jsonIn)
{
	if ((jsonIn.count("pos") > 0) &&
		(jsonIn.count("size") > 0) &&
		(jsonIn.count("freq") > 0) &&
		(jsonIn.count("min") > 0) &&
		(jsonIn.count("max") > 0))
	{
		mPosX = jsonIn["pos"][0].get<int>();
		mPosY = jsonIn["pos"][1].get<int>();
		mWidth = jsonIn["size"][0].get<int>();
		mHeight = jsonIn["size"][1].get<int>();
		mTempMin = jsonIn["min"].get<float>();
		mTempMax = jsonIn["max"].get<float>();

		mFreqVals.clear();
		for (auto it = jsonIn["freq"].begin(); it != jsonIn["freq"].end(); ++it)
		{
			mFreqVals.push_back(*it);
		}
	}
}

// PluginThermalCamClient
PluginThermalCamClient::PluginThermalCamClient()
	: PluginClient("ThermalCam")
{
}

PluginThermalCamClient::~PluginThermalCamClient()
{
}

void PluginThermalCamClient::setupData(nlohmann::json& jsonObj)
{
}

void PluginThermalCamClient::tickGUI()
{
	ImGui::BulletText("Thermal Imaging");
	// Plot the histogram
	int cIdx = mActiveIdx;
	mDataLock.lock();
	auto cNames = mROINames;
	mDataLock.unlock();
	std::array<const char*, 512> cNamesChars;
	for (size_t i = 0; i < std::min(cNamesChars.size(), cNames.size()); i++)
	{
		cNamesChars[i] = cNames[i].c_str();
	}

	if (ImGui::Combo("Region", &cIdx, cNamesChars.data(), (int)cNames.size()))
	{
		mActiveIdx = cIdx;
		mActiveROI = cNames[cIdx];
	}

	if (ImGui::InputFloat("Min", &mTempStart, 0.1f, 0.5f))
	{
	
	}
	if (ImGui::InputFloat("Max", &mTempEnd, 0.1f, 0.5f))
	{

	}

	mDataLock.lock();
	auto cThermalData = getThermalData(mActiveROI);
	if (cThermalData)
	{
		auto rangeIdx = cThermalData->getPlotRangeIdx(mTempStart, mTempEnd);
		int numItems = (rangeIdx.second - rangeIdx.first) + 1;
		ImGui::PlotHistogram(
			"Temperature",
			cThermalData->mFreqVals.data() + rangeIdx.first,
			numItems,
			0,
			nullptr,
			FLT_MAX,
			FLT_MAX,
			ImVec2(0, 100));
	}
	mDataLock.unlock();


	m_drawCnt++;
}

void PluginThermalCamClient::prepareClient(std::shared_ptr<DosClient::WsRobotClient>& sClientIn)
{
	auto startsWithStr = [this](const std::string& s, const std::string& prefix) -> bool
	{
		auto i = s.begin(), j = prefix.begin();
		for (; i != s.end() && j != prefix.end() && *i == *j;
			i++, j++);
		return j == prefix.end();
	};

	// Receive images
	sClient->registerPacketRecvFn(
		COMMAND_IMG_LIST_TYPE,
		[this, startsWithStr](BaseCommandPacket* pktIn) -> void
		{
			std::lock_guard<std::mutex> scopeLock(m_drawLock);
			if (auto imgListPkt = dynamic_cast<ImageListCommandPacket*>(pktIn))
			{
				for (auto& cImg : imgListPkt->getImages())
				{
					if (startsWithStr(cImg.label, "streamColor"))
					{
						// just streaming so no need to write to disk, just cache in memory
						mCamBytes = cImg.bytes;
						if(mStreamFn)
						{
							mStreamFn(this);
						}
					}
				}
			}
		}
	);

	// Receive status msg
	sClient->registerPacketRecvFn(
		COMMAND_STATUS_TYPE,
		[this](BaseCommandPacket* pktIn) -> void
		{
			if (auto statusPkt = dynamic_cast<StatusCommandPacket*>(pktIn))
			{
				auto jsonObj = statusPkt->getStatusJSON();
				std::lock_guard<std::mutex> scopeLock(mDataLock);
				fillAllThermalData(jsonObj);
				if (mActiveROI.empty() && (mROINames.size() > 0))
				{
					mActiveIdx = 0;
					mActiveROI = mROINames.front();
				}
			}
		}
	);
}

void PluginThermalCamClient::createCameras()
{
	if (!mCreateCamFn)
	{
		return;
	}

	// mCreateCamFn : (posX, posY, scaleX, scaleY, name, description)
	float frontWidth = 16.0f;
	float frontHeight = frontWidth * 0.75f;
	mCreateCamFn(-3.0f, 0.0f, frontWidth, frontHeight, "ThermalCamera", "Thermal Camera");
}

void PluginThermalCamClient::streamCamerasFrameStep()
{
	sClient->sendRobotMsgStr("streamFrame");
}

void PluginThermalCamClient::setStreamCameraQuality(int valIn)
{
	nlohmann::json rData;
	rData["cmd"] = "streamQuality";
	rData["quality"] = valIn;
	sendStatusPkt(rData);
	m_camQuality = valIn;
}

void PluginThermalCamClient::fillAllThermalData(nlohmann::json& jsonIn)
{
	mThermalDataMap.clear();
	mROINames.clear();
	for (auto it = jsonIn.begin(); it != jsonIn.end(); ++it)
	{
		std::string roiName = it.key();
		ThermalData newData;
		newData.fillFromJSON(it.value());
		mThermalDataMap[roiName] = newData;
		mROINames.push_back(roiName);
	}
}

PluginThermalCamClient::ThermalData* PluginThermalCamClient::getThermalData(const std::string& roiName)
{
	auto findIter = mThermalDataMap.find(roiName);
	if (findIter != mThermalDataMap.end())
	{
		return &(findIter->second);
	}
	return nullptr;
}

void PluginThermalCamClient::sendStatusPkt(nlohmann::json& jsonIn)
{
	std::unique_ptr<BaseCommandPacket> sendPkt = std::make_unique<StatusCommandPacket>();
	dynamic_cast<StatusCommandPacket*>(sendPkt.get())->getStrMsg() = jsonIn.dump();
	sClient->addMsg(sendPkt);
}

REGISTER_PLUGINCLIENT_FACTORY(PluginThermalCamClient, "ThermalCam")


} // end of namespace SpotClient