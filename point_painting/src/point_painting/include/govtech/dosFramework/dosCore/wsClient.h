#pragma once
#include <condition_variable>
#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <functional>
#include <thread>
#include <dosCore/commandPacket.h>

namespace DosClient
{
	class websocket_endpoint;

	class WsClient {
	public:
		WsClient() = default;

		void startClientThread(const std::string& address);

		void addMsg(std::unique_ptr<BaseCommandPacket>& msgIn);
		void clearMsgQueue();
		void setPacketRecvCB(std::function<void(BaseCommandPacket*)> fnIn) { m_pktReceiveCB = fnIn; }
		void registerPacketRecvFn(int msgType, std::function<void(BaseCommandPacket*)> fnIn);

		void shutdown();

		template<class T>
		static std::shared_ptr<T> startClient(
			std::string address = "ws://localhost", 
			int port = 8765, 
			std::string pemFilename = "",
			bool verifyPeer = true)
		{
			std::shared_ptr<T> sClient;
			auto runClient = [&sClient, address, port, pemFilename, verifyPeer]()
			{
				sClient = std::make_shared<T>();
				if (!pemFilename.empty()) {
					sClient->setPemFile(pemFilename);
				}
				sClient->setVerifyPeer(verifyPeer);
				sClient->startClientThread(address + ":" + std::to_string(port));
			};

			std::thread sThread(runClient);
			sThread.detach();
			std::this_thread::sleep_for(std::chrono::milliseconds(5000));
			return sClient;
		}

		void setPemFile(const std::string& filename);
		void setVerifyPeer(bool flagIn);
		bool getVerifyPeer() const;

	protected:
		void processMsgQueue();

	protected:
		std::shared_ptr<websocket_endpoint> m_endpoint;
		std::mutex m_msgLock;
		std::condition_variable m_msgQueueChanged;
		std::vector<std::unique_ptr<BaseCommandPacket>> m_msgQueue;
		int m_id = -1;
		std::function<void(BaseCommandPacket*)> m_pktReceiveCB;
		std::unordered_map<int, std::function<void(BaseCommandPacket*)>> m_pktReceiveFnMap;
		std::string m_pemFile;
		bool m_verifyPeer = true;

		bool done();
		std::mutex m_doneLock;
		bool m_done = false;
	};

}