// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <nlohmann/json.hpp>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <fstream>
#include <string>
#include <streambuf>
#include <dosCore/commandPacket.h>
#include <dosCore/commandPacketParse.h>
#include <dosCore/wsServer.h>

#if defined(_WIN32)
#include <direct.h>
#include <stdlib.h>
#include <stdio.h>
#endif

class runCmdObj {
public:
	runCmdObj(const std::string& filenameIn) {
		m_configJSON = loadCmdConfig(filenameIn);
		// Spawn the exec thread
		runThread();
	}

	void setDone(bool flagIn)
	{
		m_done = flagIn;
	}

	void printCmdList(nlohmann::json& configJSON)
	{
		std::cout << "List of Commands:" << std::endl;
		for (auto it = configJSON.begin(); it != configJSON.end(); ++it) {
			std::cout << it.key() << " : " << it.value() << std::endl;
		}
	}

	nlohmann::json getCmdListJSON() const {
		// List and return out all available commands
		std::vector<std::string> cmdList;
		for (auto it = m_configJSON.begin(); it != m_configJSON.end(); ++it) {
			cmdList.push_back(it.key());
		}
		nlohmann::json rJSON;
		rJSON["cmdList"] = cmdList;
		return rJSON;
	}

	bool execCmd(const std::string& cmdIn)
	{
		if (m_configJSON.count(cmdIn) <= 0)
		{
			std::cout << "Command: " << cmdIn << " not found! Cannot execute." << std::endl;
			return false;
		}

		std::string execCmd = m_configJSON[cmdIn].get<std::string>();
		setRunStr(execCmd);
		return true;
	}

protected:
	nlohmann::json loadCmdConfig(const std::string& filenameIn)
	{
		std::ifstream fileIn(filenameIn);
		if (!fileIn.good())
		{
			std::cout << "Could not open command config file: " << filenameIn << std::endl;
			exit(-1);
		}
		std::string configStr((std::istreambuf_iterator<char>(fileIn)),
			std::istreambuf_iterator<char>());
		nlohmann::json configJSON = nlohmann::json::parse(configStr);
		printCmdList(configJSON);
		return configJSON;
	}

	void runThread()
	{
		auto runLoop = [this]()
		{
			while (!m_done)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				std::string rStr = getRunStr();
				if (!rStr.empty())
				{
					std::cout << "Executing Command: " << rStr << std::endl;
					system(rStr.c_str()); // Remember to add a & at the end to run in the background
					setRunStr("");
				}
			}
		};

		std::thread sThread(runLoop);
		sThread.detach();
	}
	
	void setRunStr(const std::string& strIn)
	{
		std::lock_guard<std::mutex> scopeLock(m_lock);
		m_runStr = strIn;
	}

	std::string getRunStr() {
		// Grab a lock and return a copy for thread safety
		std::lock_guard<std::mutex> scopeLock(m_lock);
		return std::string(m_runStr);
	}

protected:
	std::mutex m_lock;
	std::string m_runStr;
	bool m_done = false;
	nlohmann::json m_configJSON;
};

int main()
{
	std::string cmdConfigFilename("./cmdConfig.json");
	runCmdObj cRunObj(cmdConfigFilename);
	DosServer::WsServer cServer(9022);

	cServer.registerPktFn(
		DosClient::COMMAND_STATUS_TYPE,
		[&cRunObj](DosServer::WsServer& serverIn, DosClient::BaseCommandPacket* cPacket)
		{
			if (auto statusPacket = dynamic_cast<DosClient::StatusCommandPacket*>(cPacket))
			{
				auto cJSON = statusPacket->getStatusJSON();
				if (cJSON.count("cmd") > 0)
				{
					std::string cmd = cJSON["cmd"].get<std::string>();
					if (cmd == "list")
					{
						// List and return out all available commands
						nlohmann::json rJSON = cRunObj.getCmdListJSON();
						serverIn.sendStatusPacket(rJSON.dump(4));
					}
					else if (cmd == "run")
					{
						if (cJSON.count("runType") > 0)
						{
							// Executes a command from the shell
							std::string runType = cJSON["runType"].get<std::string>();
							cRunObj.execCmd(runType);
						}
					}
				}
			}
		}
	);

	cServer.startServer();

    return 0;
}