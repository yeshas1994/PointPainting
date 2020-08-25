#include "pluginRealsenseClient.h"
#include "dosCore/commandPacket.h"
#include "dosCore/commandPacketParse.h"
#include <iomanip>
#include <imguiLib/imgui.h>

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

static bool _folderExists(const std::string& folderPath)
{
#if defined(_WIN32)
  DWORD ftyp = GetFileAttributesA(folderPath.c_str());
  if (ftyp == INVALID_FILE_ATTRIBUTES) {
    return false;  //something is wrong with your path!
  }

  if (ftyp & FILE_ATTRIBUTE_DIRECTORY) {
    return true;   // this is a directory!
  }

  return false;    // this is not a directory!
#else
	DIR* dirObj = opendir(folderPath.c_str());
	if (dirObj) {
	    /* Directory exists. */
		closedir(dirObj);
		return true;
	} 
	else if (ENOENT == errno) {
	    /* Directory does not exist. */
		return false;
	} 
	else {
	    /* opendir() failed for some other reason. */
		return false;
	}

	return false;
#endif
}

static void _createFolder(const std::string& folderPath)
{
	int nError = 0;
#if defined(_WIN32)
	nError = _mkdir(folderPath.c_str()); // can be used on Windows
#else 
	mode_t nMode = 0733; // UNIX style permissions
	nError = mkdir(sPath.c_str(), nMode); // can be used on non-Windows
#endif
	if (nError != 0) {
		// handle your error here
	}
}

void _saveBytesToFile(const std::string& filename, const std::vector<uint8_t>& bytesIn)
{
	std::ofstream outfile(filename, std::ios::out | std::ios::binary);
	outfile.write(reinterpret_cast<const char *>(bytesIn.data()), bytesIn.size());
}

std::string _formatNumberStr(int numIn)
{
	std::ostringstream out;
	out << std::internal << std::setfill('0') << std::setw(8) << numIn;
	return out.str();
}

PluginRealsenseClient::PluginRealsenseClient()
	: PluginClient("RealSense")
{
}

PluginRealsenseClient::~PluginRealsenseClient()
{
	clearRunFrameStrList();
}

void PluginRealsenseClient::setupData(nlohmann::json& jsonObj)
{
	if ((jsonObj.count("outputFolder") > 0))
	{
		m_outputFolder = jsonObj["outputFolder"].get<std::string>();
	}
}

void PluginRealsenseClient::tickGUI()
{
	// MsgStrCommandPackets: recordStart, recordStop, recordInfo
	// StatusCommandPackets ( a JSON with cmd key ): getRGBD, getCamIntrinsics
	std::array<char, 1024> rStr;
	sprintf_s(rStr.data(), rStr.size(), "%s", m_outputFolder.c_str());
	if (ImGui::InputText("Result Folder", rStr.data(), rStr.size(), ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_outputFolder = rStr.data();
	}

	if ((m_drawCnt % 120) == 0)
	{
		// Update info JSON
		sClient->sendRobotMsgStr("recordInfo");
	}

	{
		std::lock_guard<std::mutex> scopeLock(m_drawLock);
		if (!m_runFramesStrList.empty())
		{
			if (ImGui::Combo("Run", &m_runIdx, m_runIdxStrList.data(), static_cast<int>(m_runIdxStrList.size())))
			{
				// do something
			}

			if (m_runIdx >= 0)
			{
				sprintf_s(rStr.data(), rStr.size(), "Frames: %s", m_runFramesStrList[m_runIdx]);
				ImGui::BulletText(rStr.data());
				if (!m_isRetrieving) {
					if (!m_isRecording) {
						ImGui::SameLine();
						if (ImGui::Button("Retrieve")) {
							beginRetrieveFrames(m_outputFolder);
						}
					}
				}
				else {
					if (m_retrieveFrameNum > 0) {
						float cFraction = static_cast<float>(m_retrieveFrameIdx) / static_cast<float>(m_retrieveFrameNum);
						sprintf_s(rStr.data(), rStr.size(), "Downloading Frame: %d", m_retrieveFrameIdx);
						ImGui::ProgressBar(cFraction, ImVec2(-1, 0), rStr.data());
					}
				}
			}
		}

		if (!m_isRetrieving) {
			bool isInfraRed = m_infraRed;
			if (ImGui::Checkbox("InfraRed", &isInfraRed))
			{
				enableInfraRed(isInfraRed);
			}

			if (ImGui::Button(m_isRecording ? "Stop" : "Start"))
			{
				m_isRecording = !m_isRecording;
				sClient->sendRobotMsgStr(m_isRecording ? "recordStart" : "recordStop");
			}
		}
	}

	m_drawCnt++;
}

void PluginRealsenseClient::prepareClient(std::shared_ptr<DosClient::WsRobotClient>& sClientIn)
{
	auto startsWithStr = [this](const std::string& s, const std::string& prefix) -> bool
	{
		auto i = s.begin(), j = prefix.begin();
		for (; i != s.end() && j != prefix.end() && *i == *j;
			i++, j++);
		return j == prefix.end();
	};

	sClient->registerPacketRecvFn(
		COMMAND_STATUS_TYPE,
		[this](BaseCommandPacket* pktIn) -> void
		{
			std::lock_guard<std::mutex> scopeLock(m_drawLock);
			if (auto statusPkt = dynamic_cast<StatusCommandPacket*>(pktIn))
			{
				clearRunFrameStrList();
				auto jsonObj = statusPkt->getStatusJSON();
				if ((jsonObj.count("numRuns") > 0) && (jsonObj.count("runs")))
				{
					auto& runsList = jsonObj["runs"];
					for (auto it = runsList.begin(); it != runsList.end(); ++it)
					{
						auto& rObj = *it;
						addRunFrameStrList(std::to_string(rObj["frames"].get<int>()));
					}
				}
			}
		}
	);

	sClient->registerPacketRecvFn(
		COMMAND_IMG_LIST_TYPE,
		[this, startsWithStr](BaseCommandPacket* pktIn) -> void
		{
			std::lock_guard<std::mutex> scopeLock(m_drawLock);
			if (auto imgListPkt = dynamic_cast<ImageListCommandPacket*>(pktIn))
			{
				for (auto& cImg : imgListPkt->getImages())
				{
					auto resourcePath = getResourcePath(m_outputFolder, cImg.label);
					std::string fileExt;
					if (startsWithStr(cImg.label, "color"))
					{
						fileExt = ".jpg";
					}
					else if (startsWithStr(cImg.label, "depth"))
					{
						fileExt = ".png";
					}
					else if (startsWithStr(cImg.label, "trajectory"))
					{
						fileExt = ".mat";
					}
					else if (startsWithStr(cImg.label, "streamColor"))
					{
						// just streaming so no need to write to disk, just cache in memory
						mCamBytes = cImg.bytes;
						if(mStreamFn)
						{
							mStreamFn(this);
						}
					}

					if (!fileExt.empty())
					{
						_saveBytesToFile(resourcePath + _formatNumberStr(m_retrieveFrameIdx) + fileExt, cImg.bytes);
					}
				}
				m_retrieveFrameIdx++;
				if (m_retrieveFrameIdx < m_retrieveFrameNum)
				{
					requestFrameData(m_retrieveFrameIdx);
				}
				else {
					m_isRetrieving = false;
				}
			}
		}
	);
}

void PluginRealsenseClient::createCameras()
{
	if (!mCreateCamFn)
	{
		return;
	}

	// mCreateCamFn : (posX, posY, scaleX, scaleY, name, description)
	float frontWidth = 16.0f;
	float frontHeight = frontWidth * 0.75f;
	mCreateCamFn(-3.0f, 0.0f, frontWidth, frontHeight, "RealSenseCamera", "RealSense Camera");
}

void PluginRealsenseClient::streamCamerasFrameStep()
{
	sClient->sendRobotMsgStr("streamFrame");
}

void PluginRealsenseClient::setStreamCameraQuality(int valIn)
{
	nlohmann::json rData;
	rData["cmd"] = "streamQuality";
	rData["quality"] = valIn;
	sendStatusPkt(rData);
	m_camQuality = valIn;
}

bool PluginRealsenseClient::getInfraRed() const
{
	return m_infraRed;
}

void PluginRealsenseClient::enableInfraRed(bool flagIn)
{
	nlohmann::json rData;
	rData["cmd"] = "infraRed";
	rData["flag"] = (flagIn ? 1 : 0);
	sendStatusPkt(rData);
	m_infraRed = flagIn;
}

std::string PluginRealsenseClient::getResourcePath(const std::string& pathIn, const std::string& resourceType) const
{
	std::string retPath = pathIn + "/" + resourceType + "/";
	if(!_folderExists(retPath))
	{
		_createFolder(retPath);
	}
	return retPath;
}

void PluginRealsenseClient::clearRunFrameStrList()
{
	for (size_t i = 0; i < m_runFramesStrList.size(); i++)
	{
		delete m_runFramesStrList[i];
		delete m_runIdxStrList[i];
	}

	m_runFramesStrList.clear();
	m_runIdxStrList.clear();
}

void PluginRealsenseClient::addRunFrameStrList(const std::string& valIn)
{
	auto newStrBuffer = [this](const std::string& valIn) -> char*
	{
		char* buffer = new char[valIn.size() + 1]; //we need extra char for null terminator
		std::memcpy(buffer, valIn.c_str(), valIn.size() + 1);
		return buffer;
	};

	m_runFramesStrList.push_back(newStrBuffer(valIn));
	m_runIdxStrList.push_back(newStrBuffer(std::to_string(m_runIdxStrList.size())));
}

void PluginRealsenseClient::beginRetrieveFrames(const std::string& pathIn)
{
	_createFolder(pathIn);
	m_retrieveFrameNum = std::atoi(m_runFramesStrList[m_runIdx]);
	m_retrieveFrameIdx = 0;
	requestFrameData(m_retrieveFrameIdx);
	m_isRetrieving = true;
}

void PluginRealsenseClient::requestFrameData(int idx)
{
	nlohmann::json rData;
	rData["cmd"] = "getRGBD";
	rData["run"] = m_runIdx;
	rData["frame"] = m_retrieveFrameIdx;
	sendStatusPkt(rData);
}

void PluginRealsenseClient::sendStatusPkt(nlohmann::json& jsonIn)
{
	std::unique_ptr<BaseCommandPacket> sendPkt = std::make_unique<StatusCommandPacket>();
	dynamic_cast<StatusCommandPacket*>(sendPkt.get())->getStrMsg() = jsonIn.dump();
	sClient->addMsg(sendPkt);
}

REGISTER_PLUGINCLIENT_FACTORY(PluginRealsenseClient, "RealSense")

} // end of namespace SpotClient