# WebSockets Server/Client
The communication framework used here is via WebSockets given its bidirectional nature. You need a client and server ( obviously )

# wsClient

To connect to a server you use ( or you should derive from ) wsClient:

```
	class WsClient {
	public:
		WsClient() = default;

		void startClientThread(const std::string& address);

		void addMsg(std::unique_ptr<BaseCommandPacket>& msgIn);
		void clearMsgQueue();
		void setPacketRecvCB(std::function<void(BaseCommandPacket*)> fnIn) { m_pktReceiveCB = fnIn; }
		void registerPacketRecvFn(int msgType, std::function<void(BaseCommandPacket*)> fnIn);

		template<class T>
		static std::shared_ptr<T> startClient(std::string address = "ws://localhost", int port = 8765)
		{
			std::shared_ptr<T> sClient;
			auto startClient = [&sClient, address, port]()
			{
				sClient = std::make_shared<T>();
				sClient->startClientThread(address + ":" + std::to_string(port));

				while (true)
				{
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}
			};

			std::thread sThread(startClient);
			sThread.detach();
			std::this_thread::sleep_for(std::chrono::milliseconds(2000));
			return sClient;
		}

		void setPemFile(const std::string& filename);

	protected:
		void processMsgQueue();

	protected:
		std::shared_ptr<websocket_endpoint> m_endpoint;
		std::mutex m_msgLock;
		std::vector<std::unique_ptr<BaseCommandPacket>> m_msgQueue;
		int m_id = -1;
		std::function<void(BaseCommandPacket*)> m_pktReceiveCB;
		std::unordered_map<int, std::function<void(BaseCommandPacket*)>> m_pktReceiveFnMap;
		std::string m_pemFile;
	};
```

The correct way to instantiate your client is via:
```
    auto newClient = WsClient::startClient<MyWsClient>("ws://localhost", 9000); // Address + Port
```

To send a message which is packed as a commandPacket, you do:
```
    newClient->addMsg(MyCmdPkt); // Put onto msg queue, sent out in background thread
```

To listen to for incoming commandPackets, you register:
```
    newClient->registerPacketRecvFn(MY_PACKET_TYPE, [this](BaseCommandPacket* pktIn) {
        auto realPkt = dynamic_cast<MyCommandPacket *>(pktIn);
        if(realPkt)
        {
            // Use realPkt
        }
    });
```

# wsServer
