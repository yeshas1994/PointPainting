#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <thread>
#include <mutex>
#include <functional>
#include <websocketpp/client.hpp>
#include <websocketpp/server.hpp>

#if defined(DOS_TLS_ENABLE)
#include <websocketpp/config/asio.hpp>
#else
#include <websocketpp/config/asio_no_tls.hpp>
#endif

#include <dosCore/commandPacket.h>

namespace DosServer
{
#if defined(DOS_TLS_ENABLE)
typedef websocketpp::server<websocketpp::config::asio_tls> server;
typedef websocketpp::lib::shared_ptr<websocketpp::lib::asio::ssl::context> ssl_context_ptr;
#else
typedef websocketpp::server<websocketpp::config::asio> server;
#endif

typedef websocketpp::client<websocketpp::config::asio> client;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

// See https://wiki.mozilla.org/Security/Server_Side_TLS for more details about
// the TLS modes.
enum class tls_mode {
	MOZILLA_INTERMEDIATE = 1,
	MOZILLA_MODERN = 2
};

// pull out the type of messages sent by our config
typedef server::message_ptr message_ptr;

// A websockets server
class WsServer {
public:
	WsServer(int portIn=9002);

	// Starts a new server thread
	void startServer();

	// Stops the server thread
	void stopServer();

	// Sets the data callback handler
	void setDataCB(std::function<void(WsServer& , const std::string&)> cbIn);

	// Register a callback handler based on a CommandPacket type
	void registerPktFn(int msgType, std::function<void(WsServer&, DosClient::BaseCommandPacket*)> fnIn);

	// Sending messages
	void addMsg(std::unique_ptr<DosClient::BaseCommandPacket>& msgIn);
	void clearMsgQueue();
	void processMsgQueue(websocketpp::connection_hdl hdl);
	void sendMsgPacket(const std::string& msgIn);
	void sendStatusPacket(const std::string& jsonPayload);

	// Set only allow LocalHost connections
	void setOnlyLocalHost(bool flagIn);
	bool getOnlyLocalHost() const { return m_onlyLocalHostClients; }

public:
	server* m_runServer;

protected:
	void run();

	void on_open(websocketpp::connection_hdl hdl);
	void on_close(websocketpp::connection_hdl hdl);
	bool on_validate(websocketpp::connection_hdl hdl);
	void on_message(server* s, websocketpp::connection_hdl hdl, message_ptr msg);
#if defined(DOS_TLS_ENABLE)
	ssl_context_ptr on_tls_init(tls_mode mode, websocketpp::connection_hdl hdl);

	std::string get_password() const;
	void setPemFile(const std::string& filename);
	void setDhFile(const std::string& filename);
#endif

protected:
	std::mutex m_lock;
	int m_port = 0;
	bool m_isRunning = false;
	std::function<void(WsServer&, const std::string&)> m_dataCB;
	std::vector<std::unique_ptr<DosClient::BaseCommandPacket>> m_msgQueue;
	std::unordered_map<int, std::function<void(WsServer&, DosClient::BaseCommandPacket*)>> m_pktReceiveFnMap;
	std::string m_pemFile;
	std::string m_dhFile;
	bool m_onlyLocalHostClients = false;
};

} // end of namespace SpotServer