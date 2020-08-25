#define _WEBSOCKETPP_CPP11_RANDOM_DEVICE_
#include <dosCore/wsClient.h>
#if defined(DOS_TLS_ENABLE)
#include <websocketpp/config/asio_client.hpp>
#else
#include <websocketpp/config/asio_no_tls_client.hpp>
#endif

#include <websocketpp/client.hpp>

#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>
#include <dosCore/commandPacketParse.h>
#include <msgpack11.hpp>
#include <stb_image.h>

#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <functional>

#ifdef _MSC_VER 
//not #if defined(_WIN32) || defined(_WIN64) because we have strncasecmp in mingw
#define strncasecmp _strnicmp
#define strcasecmp _stricmp
#endif

namespace DosClient
{

#if defined(DOS_TLS_ENABLE)
	typedef websocketpp::client<websocketpp::config::asio_tls_client> client;
	typedef websocketpp::lib::shared_ptr<websocketpp::lib::asio::ssl::context> context_ptr;
#else
	typedef websocketpp::client<websocketpp::config::asio_client> client;
#endif

	class connection_metadata {
	public:
		typedef websocketpp::lib::shared_ptr<connection_metadata> ptr;

		connection_metadata(int id, websocketpp::connection_hdl hdl, std::string uri)
			: m_id(id)
			, m_hdl(hdl)
			, m_status("Connecting")
			, m_uri(uri)
			, m_server("N/A")
		{}

		void on_open(client* c, websocketpp::connection_hdl hdl) {
			m_status = "Open";

			client::connection_ptr con = c->get_con_from_hdl(hdl);
			m_server = con->get_response_header("Server");
		}

		void on_fail(client* c, websocketpp::connection_hdl hdl) {
			m_status = "Failed";

			client::connection_ptr con = c->get_con_from_hdl(hdl);
			m_server = con->get_response_header("Server");
			m_error_reason = con->get_ec().message();
		}

		void on_close(client* c, websocketpp::connection_hdl hdl) {
			m_status = "Closed";
			client::connection_ptr con = c->get_con_from_hdl(hdl);
			std::stringstream s;
			s << "close code: " << con->get_remote_close_code() << " ("
				<< websocketpp::close::status::get_string(con->get_remote_close_code())
				<< "), close reason: " << con->get_remote_close_reason();
			m_error_reason = s.str();
		}

		void on_message(websocketpp::connection_hdl, client::message_ptr msg) {
			if (m_onDataCB == nullptr)
			{
				return;
			}
			m_onDataCB(msg->get_payload());
		}

		websocketpp::connection_hdl get_hdl() const {
			return m_hdl;
		}

		int get_id() const {
			return m_id;
		}

		std::string get_status() const {
			return m_status;
		}

		void setOnDataCB(std::function<void(const std::string&)> fnIn)
		{
			m_onDataCB = fnIn;
		}

		friend std::ostream& operator<< (std::ostream& out, connection_metadata const& data);
	private:
		int m_id;
		websocketpp::connection_hdl m_hdl;
		std::string m_status;
		std::string m_uri;
		std::string m_server;
		std::string m_error_reason;
		std::function<void(const std::string&)> m_onDataCB;
	};

	std::ostream& operator<< (std::ostream& out, connection_metadata const& data) {
		out << "> URI: " << data.m_uri << "\n"
			<< "> Status: " << data.m_status << "\n"
			<< "> Remote Server: " << (data.m_server.empty() ? "None Specified" : data.m_server) << "\n"
			<< "> Error/close reason: " << (data.m_error_reason.empty() ? "N/A" : data.m_error_reason) << "\n";

		return out;
	}

	class websocket_endpoint {
	public:
		websocket_endpoint() : m_next_id(0) {
			m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
			m_endpoint.set_error_channels(websocketpp::log::elevel::all);

			// For more verbose logging:
			//m_endpoint.set_access_channels(websocketpp::log::alevel::all);
			//m_endpoint.clear_access_channels(websocketpp::log::alevel::frame_payload);
			//m_endpoint.set_error_channels(websocketpp::log::elevel::all);

			m_endpoint.init_asio();
			m_endpoint.start_perpetual();

			m_thread = websocketpp::lib::make_shared<websocketpp::lib::thread>(&client::run, &m_endpoint);
		}

		~websocket_endpoint() {
			m_endpoint.stop_perpetual();

			for (con_list::const_iterator it = m_connection_list.begin(); it != m_connection_list.end(); ++it) {
				if (it->second->get_status() != "Open") {
					// Only close open connections
					continue;
				}

				std::cout << "> Closing connection " << it->second->get_id() << std::endl;

				websocketpp::lib::error_code ec;
				m_endpoint.close(it->second->get_hdl(), websocketpp::close::status::going_away, "", ec);
				if (ec) {
					std::cout << "> Error closing connection " << it->second->get_id() << ": "
						<< ec.message() << std::endl;
				}
			}

			m_thread->join();
		}

#if defined(DOS_TLS_ENABLE)
		/// Verify that one of the subject alternative names matches the given hostname
		static bool verifySubjectAlternativeName(const char* hostname, X509* cert) {
			STACK_OF(GENERAL_NAME)* san_names = NULL;

			san_names = (STACK_OF(GENERAL_NAME)*) X509_get_ext_d2i(cert, NID_subject_alt_name, NULL, NULL);
			if (san_names == NULL) {
				return false;
			}

			int san_names_count = sk_GENERAL_NAME_num(san_names);

			bool result = false;

			for (int i = 0; i < san_names_count; i++) {
				const GENERAL_NAME* current_name = sk_GENERAL_NAME_value(san_names, i);

				if (current_name->type != GEN_DNS) {
					continue;
				}

				char* dns_name = (char*)ASN1_STRING_data(current_name->d.dNSName);

				// Make sure there isn't an embedded NUL character in the DNS name
				if (ASN1_STRING_length(current_name->d.dNSName) != strlen(dns_name)) {
					break;
				}
				// Compare expected hostname with the CN
				result = (strcasecmp(hostname, dns_name) == 0);
			}
			sk_GENERAL_NAME_pop_free(san_names, GENERAL_NAME_free);

			return result;
		}

		/// Verify that the certificate common name matches the given hostname
		static bool verifyCommonName(const char* hostname, X509* cert) {
			// Find the position of the CN field in the Subject field of the certificate
			int common_name_loc = X509_NAME_get_index_by_NID(X509_get_subject_name(cert), NID_commonName, -1);
			if (common_name_loc < 0) {
				return false;
			}

			// Extract the CN field
			X509_NAME_ENTRY* common_name_entry = X509_NAME_get_entry(X509_get_subject_name(cert), common_name_loc);
			if (common_name_entry == NULL) {
				return false;
			}

			// Convert the CN field to a C string
			ASN1_STRING* common_name_asn1 = X509_NAME_ENTRY_get_data(common_name_entry);
			if (common_name_asn1 == NULL) {
				return false;
			}

			char* common_name_str = (char*)ASN1_STRING_data(common_name_asn1);

			// Make sure there isn't an embedded NUL character in the CN
			if (ASN1_STRING_length(common_name_asn1) != strlen(common_name_str)) {
				return false;
			}

			// Compare expected hostname with the CN
			return (strcasecmp(hostname, common_name_str) == 0);
		}

		static bool verifyCertificate(const char* hostname, bool preverified, asio::ssl::verify_context& ctx) {
			// The verify callback can be used to check whether the certificate that is
			// being presented is valid for the peer. For example, RFC 2818 describes
			// the steps involved in doing this for HTTPS. Consult the OpenSSL
			// documentation for more details. Note that the callback is called once
			// for each certificate in the certificate chain, starting from the root
			// certificate authority.

			// Retrieve the depth of the current cert in the chain. 0 indicates the
			// actual server cert, upon which we will perform extra validation
			// (specifically, ensuring that the hostname matches. For other certs we
			// will use the 'preverified' flag from Asio, which incorporates a number of
			// non-implementation specific OpenSSL checking, such as the formatting of
			// certs and the trusted status based on the CA certs we imported earlier.
			int depth = X509_STORE_CTX_get_error_depth(ctx.native_handle());

			// if we are on the final cert and everything else checks out, ensure that
			// the hostname is present on the list of SANs or the common name (CN).
			if (depth == 0 && preverified) {
				X509* cert = X509_STORE_CTX_get_current_cert(ctx.native_handle());

				if (verifySubjectAlternativeName(hostname, cert)) {
					return true;
				}
				else if (verifyCommonName(hostname, cert)) {
					return true;
				}
				else {
					return false;
				}
			}

			return preverified;
		}

		/// TLS Initialization handler
		/**
		 * WebSocket++ core and the Asio Transport do not handle TLS context creation
		 * and setup. This callback is provided so that the end user can set up their
		 * TLS context using whatever settings make sense for their application.
		 *
		 * As Asio and OpenSSL do not provide great documentation for the very common
		 * case of connect and actually perform basic verification of server certs this
		 * example includes a basic implementation (using Asio and OpenSSL) of the
		 * following reasonable default settings and verification steps:
		 *
		 * - Disable SSLv2 and SSLv3
		 * - Load trusted CA certificates and verify the server cert is trusted.
		 * - Verify that the hostname matches either the common name or one of the
		 *   subject alternative names on the certificate.
		 *
		 * This is not meant to be an exhaustive reference implimentation of a perfect
		 * TLS client, but rather a reasonable starting point for building a secure
		 * TLS encrypted WebSocket client.
		 *
		 * If any TLS, Asio, or OpenSSL experts feel that these settings are poor
		 * defaults or there are critically missing steps please open a GitHub issue
		 * or drop a line on the project mailing list.
		 *
		 * Note the bundled CA cert ca-chain.cert.pem is the CA cert that signed the
		 * cert bundled with echo_server_tls. You can use print_client_tls with this
		 * CA cert to connect to echo_server_tls as long as you use /etc/hosts or
		 * something equivilent to spoof one of the names on that cert
		 * (websocketpp.org, for example).
		 */
		static context_ptr on_tls_init(
			std::string hostname, 
			std::string pemFile, 
			bool verifyPeer,
			websocketpp::connection_hdl) 
		{
			context_ptr ctx = websocketpp::lib::make_shared<asio::ssl::context>(asio::ssl::context::sslv23);

			try {
				ctx->set_options(asio::ssl::context::default_workarounds |
					asio::ssl::context::no_sslv2 |
					asio::ssl::context::no_sslv3 |
					asio::ssl::context::single_dh_use);

				ctx->set_verify_mode(verifyPeer ? asio::ssl::verify_peer : asio::ssl::verify_none);
				ctx->set_verify_callback(bind(
					&websocket_endpoint::verifyCertificate,
					hostname.c_str(), 
					websocketpp::lib::placeholders::_1, 
					websocketpp::lib::placeholders::_2));

				// Here we load the CA certificates of all CA's that this client trusts.
				ctx->load_verify_file(pemFile.c_str());
			}
			catch (std::exception & e) {
				std::cout << e.what() << std::endl;
			}
			return ctx;
		}
#endif
		int connect(std::string const& uri, bool verifyPeer) 
		{
			websocketpp::lib::error_code ec;

#if defined(DOS_TLS_ENABLE)
			// Extract hostname from format: wss://HOSTNAME:PORT
			std::size_t pTokenIdx = uri.find_last_of(":");
			std::size_t pSlashIdx = uri.find_last_of("/") + 1;
			std::string cHostname = uri.substr(pSlashIdx, uri.length() - pSlashIdx - (uri.length() - pTokenIdx));
			if (!verifyPeer)
			{
				std::cout << "WebSockets Client is NOT Verifying Peer CA." << std::endl;
			}

			m_endpoint.set_tls_init_handler(websocketpp::lib::bind(
				&websocket_endpoint::on_tls_init,
				cHostname,
				m_pemFile,
				verifyPeer ? true : false,
				websocketpp::lib::placeholders::_1));
#endif
			client::connection_ptr con = m_endpoint.get_connection(uri, ec);

			if (ec) {
				std::cout << "> Connect initialization error: " << ec.message() << std::endl;
				return -1;
			}

			int new_id = m_next_id++;
			connection_metadata::ptr metadata_ptr = websocketpp::lib::make_shared<connection_metadata>(new_id, con->get_handle(), uri);
			m_connection_list[new_id] = metadata_ptr;

			con->set_open_handler(websocketpp::lib::bind(
				&connection_metadata::on_open,
				metadata_ptr,
				&m_endpoint,
				websocketpp::lib::placeholders::_1
			));
			con->set_fail_handler(websocketpp::lib::bind(
				&connection_metadata::on_fail,
				metadata_ptr,
				&m_endpoint,
				websocketpp::lib::placeholders::_1
			));
			con->set_close_handler(websocketpp::lib::bind(
				&connection_metadata::on_close,
				metadata_ptr,
				&m_endpoint,
				websocketpp::lib::placeholders::_1
			));
			con->set_message_handler(websocketpp::lib::bind(
				&connection_metadata::on_message,
				metadata_ptr,
				websocketpp::lib::placeholders::_1,
				websocketpp::lib::placeholders::_2
			));

			m_endpoint.connect(con);

			return new_id;
		}

		void close(int id, websocketpp::close::status::value code, std::string reason) {
			websocketpp::lib::error_code ec;

			con_list::iterator metadata_it = m_connection_list.find(id);
			if (metadata_it == m_connection_list.end()) {
				std::cout << "> No connection found with id " << id << std::endl;
				return;
			}

			m_endpoint.close(metadata_it->second->get_hdl(), code, reason, ec);
			if (ec) {
				std::cout << "> Error initiating close: " << ec.message() << std::endl;
			}
		}

		void send(int id, std::string message, bool isBinary=false) 
		{
			websocketpp::lib::error_code ec;

			con_list::iterator metadata_it = m_connection_list.find(id);
			if (metadata_it == m_connection_list.end()) {
				std::cout << "> No connection found with id " << id << std::endl;
				return;
			}

			//std::cout << "Trying to send message of size: " << message.length() << std::endl;
			m_endpoint.send(
				metadata_it->second->get_hdl(), 
				message, 
				isBinary ? websocketpp::frame::opcode::binary : websocketpp::frame::opcode::text,
				ec);

			if (ec) {
				std::cout << "> Error sending message: " << ec.message() << std::endl;
				return;
			}
		}

		connection_metadata::ptr get_metadata(int id) const {
			con_list::const_iterator metadata_it = m_connection_list.find(id);
			if (metadata_it == m_connection_list.end()) {
				return connection_metadata::ptr();
			}
			else {
				return metadata_it->second;
			}
		}

	public:
		std::string m_pemFile;

	private:
		typedef std::map<int, connection_metadata::ptr> con_list;

		client m_endpoint;
		websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;

		con_list m_connection_list;
		int m_next_id;
	};

	// WsClient
	void WsClient::startClientThread(const std::string& address)
	{
		std::string input;
		// Initiate connection
		m_endpoint = std::make_shared<websocket_endpoint>();
		m_endpoint->m_pemFile = m_pemFile;

		m_id = m_endpoint->connect(address, getVerifyPeer());
		if (m_id != -1) {
			std::cout << "> Created connection with id " << m_id << " at: " << address << std::endl;
		}
		else {
			std::cout << "> Connection failed at: " << address << std::endl;
			return;
		}

		// Setup data callbacks
		m_endpoint->get_metadata(m_id)->setOnDataCB(
			[this](const std::string& dataIn) {
				auto respPacket = CommandPacketMgr::getCommandPacket(dataIn);
				if (m_pktReceiveCB)
				{
					m_pktReceiveCB(respPacket.get());
				}

				auto fnIter = m_pktReceiveFnMap.find(respPacket->getPacketTypeID());
				if(fnIter != m_pktReceiveFnMap.end())
				{
					auto& pktFn = fnIter->second;
					pktFn(respPacket.get());
				}
			}
		);

		// Now event loop
		while (!done()) {
			processMsgQueue();
			std::unique_lock<std::mutex> lock( m_msgLock );
			m_msgQueueChanged.wait( lock, [ this ] {return this->m_msgQueue.size() > 0; } );
		}
	}

	void WsClient::addMsg(std::unique_ptr<BaseCommandPacket>& msgIn)
	{
		std::lock_guard<std::mutex> scopeLock(m_msgLock);
		m_msgQueue.push_back(std::move(msgIn));
		m_msgQueueChanged.notify_one();
	}

	void WsClient::clearMsgQueue()
	{
		std::lock_guard<std::mutex> scopeLock(m_msgLock);
		m_msgQueue.clear();
	}

	void WsClient::setPemFile(const std::string& filename)
	{
		m_pemFile = filename;
	}

	void WsClient::setVerifyPeer(bool flagIn)
	{
		m_verifyPeer = flagIn;
	}

	bool WsClient::getVerifyPeer() const
	{
		return m_verifyPeer;
	}

	void WsClient::processMsgQueue()
	{
		
		std::lock_guard<std::mutex> scopeLock( m_msgLock );
		for ( auto& cMsg : m_msgQueue )
		{
			m_endpoint->send( m_id, cMsg->encode(), true );
		}
		m_msgQueue.clear();
	}

	void WsClient::registerPacketRecvFn(int msgType, std::function<void(BaseCommandPacket*)> fnIn)
	{
		m_pktReceiveFnMap[msgType] = fnIn;
	}

	void WsClient::shutdown()
	{
		processMsgQueue(); // flush all remaining packets

		std::lock_guard<std::mutex> scopeLock(m_doneLock);
		m_done = true;
	}

	bool WsClient::done()
	{
		std::lock_guard<std::mutex> scopeLock(m_doneLock);
		return m_done;
	}

} // end of namespace SpotClient