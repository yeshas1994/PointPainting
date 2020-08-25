#include "wsServer.h"
#include <dosCore/commandPacketParse.h>
#include <iostream>

namespace DosServer
{

WsServer::WsServer(int portIn)
{
	m_port = portIn;
}

void WsServer::startServer()
{
	std::thread cThread(&WsServer::run, this);
	cThread.join();
}

void WsServer::stopServer()
{
	m_runServer->stop_listening();
}

void WsServer::setDataCB(std::function<void(WsServer&, const std::string&)> cbIn)
{
	m_dataCB = cbIn;
}

void WsServer::registerPktFn(int msgType, std::function<void(WsServer&, DosClient::BaseCommandPacket*)> fnIn)
{
	m_pktReceiveFnMap[msgType] = fnIn;
}

void WsServer::addMsg(std::unique_ptr<DosClient::BaseCommandPacket>& msgIn)
{
	std::lock_guard<std::mutex> scopeLock(m_lock);
	m_msgQueue.push_back(std::move(msgIn));
}

void WsServer::clearMsgQueue()
{
	std::lock_guard<std::mutex> scopeLock(m_lock);
	m_msgQueue.clear();
}

void WsServer::processMsgQueue(websocketpp::connection_hdl hdl)
{
	std::lock_guard<std::mutex> scopeLock(m_lock);
	for (auto& cMsg : m_msgQueue)
	{
		m_runServer->send(hdl, cMsg->encode(), websocketpp::frame::opcode::binary);
	}
	m_msgQueue.clear();
}

void WsServer::sendMsgPacket(const std::string& msgIn)
{
	std::unique_ptr<DosClient::BaseCommandPacket> newPkt = std::make_unique<DosClient::MsgStrCommandPacket>();
	dynamic_cast<DosClient::MsgStrCommandPacket*>(newPkt.get())->getStrMsg() = msgIn;
	addMsg(newPkt);
}

void WsServer::sendStatusPacket(const std::string& jsonPayload)
{
	std::unique_ptr<DosClient::BaseCommandPacket> newPkt = std::make_unique<DosClient::StatusCommandPacket>();
	dynamic_cast<DosClient::StatusCommandPacket*>(newPkt.get())->getStrMsg() = jsonPayload;
	addMsg(newPkt);
}

void WsServer::setOnlyLocalHost(bool flagIn)
{
	m_onlyLocalHostClients = flagIn;
}

// Callback when a new client joins
void WsServer::on_open(websocketpp::connection_hdl hdl)
{
	std::cout << "A new client has connected." << std::endl;
}

// Callback when a client leaves
void WsServer::on_close(websocketpp::connection_hdl hdl)
{
	std::cout << "A client has disconnected." << std::endl;
}

// Callback for client validation
bool WsServer::on_validate(websocketpp::connection_hdl hdl)
{
	auto cCon = m_runServer->get_con_from_hdl(hdl);
	std::cout << "Client Host is: " << cCon->get_host() << std::endl;
	if (getOnlyLocalHost())
	{
		std::cout << "Only LocalHost connections are allowed!" << std::endl;
		if (cCon->get_host() != "localhost")
		{
			std::cout << "Incoming connection is not localhost so rejecting." << std::endl;
			return false;
		}
		else {
			std::cout << "Incoming connection localhost is accepted." << std::endl;
		}
	}

	std::cout << "Client is validated" << std::endl;
	return true;
}

// Define a callback to handle incoming messages
void WsServer::on_message(server* s, websocketpp::connection_hdl hdl, message_ptr msg) {
	/*
	std::cout << "on_message called with hdl: " << hdl.lock().get()
		<< " and message: " << msg->get_payload()
		<< std::endl;

	// check for a special command to instruct the server to stop listening so
	// it can be cleanly exited.
	if (msg->get_payload() == "stop-listening") {
		s->stop_listening();
		return;
	}
	*/

	if (m_dataCB)
	{
		m_dataCB(*this, msg->get_payload());
	}

	auto cPacket = DosClient::CommandPacketMgr::getCommandPacket(msg->get_payload());
	auto fnIter = m_pktReceiveFnMap.find(cPacket->getPacketTypeID());
	if(fnIter != m_pktReceiveFnMap.end())
	{
		auto& pktFn = fnIter->second;
		pktFn(*this, cPacket.get());
	}

	processMsgQueue(hdl);

	/*
	try {
		s->send(hdl, msg->get_payload(), msg->get_opcode());
	}
	catch (websocketpp::exception const& e) {
		std::cout << "WsServer failed because: "
			<< "(" << e.what() << ")" << std::endl;
	}
	*/
}

#if defined(DOS_TLS_ENABLE)
ssl_context_ptr WsServer::on_tls_init(tls_mode mode, websocketpp::connection_hdl hdl)
{
	namespace asio = websocketpp::lib::asio;

	std::cout << "on_tls_init called with hdl: " << hdl.lock().get() << std::endl;
	std::cout << "using TLS mode: " << (mode == tls_mode::MOZILLA_MODERN ? "Mozilla Modern" : "Mozilla Intermediate") << std::endl;

	ssl_context_ptr ctx = websocketpp::lib::make_shared<asio::ssl::context>(asio::ssl::context::sslv23);

	try {
		if (mode == tls_mode::MOZILLA_MODERN) {
			// Modern disables TLSv1
			ctx->set_options(asio::ssl::context::default_workarounds |
				asio::ssl::context::no_sslv2 |
				asio::ssl::context::no_sslv3 |
				asio::ssl::context::no_tlsv1 |
				asio::ssl::context::single_dh_use);
		}
		else {
			ctx->set_options(asio::ssl::context::default_workarounds |
				asio::ssl::context::no_sslv2 |
				asio::ssl::context::no_sslv3 |
				asio::ssl::context::single_dh_use);
		}
		ctx->set_password_callback(bind(&WsServer::get_password, this));
		ctx->use_certificate_chain_file(m_pemFile);
		ctx->use_private_key_file(m_pemFile, asio::ssl::context::pem);

		// Example method of generating this file:
		// `openssl dhparam -out dh.pem 2048`
		// Mozilla Intermediate suggests 1024 as the minimum size to use
		// Mozilla Modern suggests 2048 as the minimum size to use.
		// https://security.stackexchange.com/questions/38206/can-someone-explain-what-exactly-is-accomplished-by-generation-of-dh-parameters
		if (!m_dhFile.empty()) {
			ctx->use_tmp_dh_file(m_dhFile);
		}

		std::string ciphers;

		if (mode == tls_mode::MOZILLA_MODERN) {
			ciphers = "ECDHE-RSA-AES128-GCM-SHA256:ECDHE-ECDSA-AES128-GCM-SHA256:ECDHE-RSA-AES256-GCM-SHA384:ECDHE-ECDSA-AES256-GCM-SHA384:DHE-RSA-AES128-GCM-SHA256:DHE-DSS-AES128-GCM-SHA256:kEDH+AESGCM:ECDHE-RSA-AES128-SHA256:ECDHE-ECDSA-AES128-SHA256:ECDHE-RSA-AES128-SHA:ECDHE-ECDSA-AES128-SHA:ECDHE-RSA-AES256-SHA384:ECDHE-ECDSA-AES256-SHA384:ECDHE-RSA-AES256-SHA:ECDHE-ECDSA-AES256-SHA:DHE-RSA-AES128-SHA256:DHE-RSA-AES128-SHA:DHE-DSS-AES128-SHA256:DHE-RSA-AES256-SHA256:DHE-DSS-AES256-SHA:DHE-RSA-AES256-SHA:!aNULL:!eNULL:!EXPORT:!DES:!RC4:!3DES:!MD5:!PSK";
		}
		else {
			ciphers = "ECDHE-RSA-AES128-GCM-SHA256:ECDHE-ECDSA-AES128-GCM-SHA256:ECDHE-RSA-AES256-GCM-SHA384:ECDHE-ECDSA-AES256-GCM-SHA384:DHE-RSA-AES128-GCM-SHA256:DHE-DSS-AES128-GCM-SHA256:kEDH+AESGCM:ECDHE-RSA-AES128-SHA256:ECDHE-ECDSA-AES128-SHA256:ECDHE-RSA-AES128-SHA:ECDHE-ECDSA-AES128-SHA:ECDHE-RSA-AES256-SHA384:ECDHE-ECDSA-AES256-SHA384:ECDHE-RSA-AES256-SHA:ECDHE-ECDSA-AES256-SHA:DHE-RSA-AES128-SHA256:DHE-RSA-AES128-SHA:DHE-DSS-AES128-SHA256:DHE-RSA-AES256-SHA256:DHE-DSS-AES256-SHA:DHE-RSA-AES256-SHA:AES128-GCM-SHA256:AES256-GCM-SHA384:AES128-SHA256:AES256-SHA256:AES128-SHA:AES256-SHA:AES:CAMELLIA:DES-CBC3-SHA:!aNULL:!eNULL:!EXPORT:!DES:!RC4:!MD5:!PSK:!aECDH:!EDH-DSS-DES-CBC3-SHA:!EDH-RSA-DES-CBC3-SHA:!KRB5-DES-CBC3-SHA";
		}

		if (SSL_CTX_set_cipher_list(ctx->native_handle(), ciphers.c_str()) != 1) {
			std::cout << "Error setting cipher list" << std::endl;
		}
	}
	catch (std::exception & e) {
		std::cout << "Exception: " << e.what() << std::endl;
	}
	return ctx;
}

std::string WsServer::get_password() const
{
	return std::string("");
}

void WsServer::setPemFile(const std::string& filename)
{
	m_pemFile = filename;
}

void WsServer::setDhFile(const std::string& filename)
{
	m_dhFile = filename;
}
#endif

void WsServer::run()
{
	// Create a server endpoint
	server newServer;
	m_runServer = &newServer;

	try {
		// Set logging settings
		newServer.set_access_channels(websocketpp::log::alevel::all);
		newServer.clear_access_channels(websocketpp::log::alevel::frame_payload);

		// Initialize Asio
		newServer.init_asio();

		// Register our message handler
		newServer.set_message_handler(
			std::bind(&WsServer::on_message, this, &newServer, std::placeholders::_1, std::placeholders::_2));

		// Register other handlers
		newServer.set_open_handler([this](websocketpp::connection_hdl hdl) {
			on_open(hdl);
		});
	
		newServer.set_validate_handler([this](websocketpp::connection_hdl hdl) -> bool {
			return on_validate(hdl);
		});

		newServer.set_close_handler([this](websocketpp::connection_hdl hdl) {
			on_close(hdl);
		});

#if defined(DOS_TLS_ENABLE)
		newServer.set_tls_init_handler(bind(&WsServer::on_tls_init, this, tls_mode::MOZILLA_INTERMEDIATE, std::placeholders::_1));
#endif

		// Listen on port
		newServer.listen(m_port);
		std::cout << "Starting WsServer on port: " << m_port << std::endl;

		// Start the server accept loop
		newServer.start_accept();

		// Start the ASIO io_service run loop
		newServer.run();
	}
	catch (websocketpp::exception const& e) {
		std::cout << e.what() << std::endl;
	}
	catch (...) {
		std::cout << "other exception" << std::endl;
	}
}

} // end of namespace SpotServer