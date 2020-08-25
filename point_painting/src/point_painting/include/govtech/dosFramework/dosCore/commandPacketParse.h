#pragma once
#include <msgpack11.hpp>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <functional>


namespace DosClient
{
	class BaseCommandPacket;

#define REGISTER_PACKET_FACTORY(TYPE) static CommandPacketFactoryBuild<TYPE> xx_##TYPE; 

	class CommandPacketMgr
	{
	public:
		using FactoryMap = std::unordered_map<int, std::function<std::unique_ptr<BaseCommandPacket>()>>;

	public:
		static FactoryMap& getFactoryMap();

		static void registerFactory(int cmdType, std::function<std::unique_ptr<BaseCommandPacket>()> fnIn);

		static std::unique_ptr<BaseCommandPacket> getCommandPacket(const std::string& rawData);
	};

	template<typename CreateClass>
	class CommandPacketFactoryBuild
	{
	public:
		CommandPacketFactoryBuild()
		{
			auto createFn = []() -> std::unique_ptr<BaseCommandPacket>
			{
				return std::make_unique<CreateClass>();
			};

			auto newPkt = createFn();
			CommandPacketMgr::registerFactory(newPkt->getPacketTypeID(), createFn);
		}
	};
} // end of namespace SpotClient