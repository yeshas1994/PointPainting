#include <dosCore/wsClient.h>
#include <dosCore/commandPacket.h>
#include <dosCore/commandPacketParse.h>
#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <fstream>
#include <streambuf>
#include <string>
#include <chrono>

// Take NOTE: If you are trying to debug TLS Websocket connections, modify:
// lima_controls/websocketpp/websocketpp/transport/asio/security/tls.hpp:
/*
	void handle_init(init_handler callback,lib::asio::error_code const & ec) {
		if (ec) {
			// Showing better errors with reference to: https://github.com/zaphoyd/websocketpp/issues/869
			//m_ec = socket::make_error_code(socket::error::tls_handshake_failed);
			m_ec = ec;
		} else {
			m_ec = lib::error_code();
		}

		callback(m_ec);
	}

*/

int main()
{
//	std::string pemFilename = "E:/lima_controls/wssUtils/testCerts/ca-chain.cert.pem";
	std::string pemFilename = "E:/lima_controls/wssUtils/testCerts/serverDefault.pem";
	auto cClient = DosClient::WsClient::startClient<DosClient::WsClient>(
		"wss://localhost", 
		8765, 
		pemFilename,
		false);

	while (true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		std::unique_ptr<DosClient::BaseCommandPacket> newPkt = std::make_unique<DosClient::MsgStrCommandPacket>();
		dynamic_cast<DosClient::MsgStrCommandPacket*>(newPkt.get())->getStrMsg() = "Hello Test Message!";
		cClient->addMsg(newPkt);
	}

	return 0;
}