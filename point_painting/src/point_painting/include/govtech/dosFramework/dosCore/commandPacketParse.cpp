#include <dosCore/commandPacketParse.h>
#include <dosCore/commandPacket.h>
#include <iostream>

namespace DosClient
{
	// CommandPacketMgr
	CommandPacketMgr::FactoryMap& CommandPacketMgr::getFactoryMap()
	{
		static FactoryMap classRegister{};
		return classRegister;
	}

	void CommandPacketMgr::registerFactory(int cmdType, std::function<std::unique_ptr<BaseCommandPacket>()> fnIn)
	{
		if(getFactoryMap().find(cmdType) != getFactoryMap().end())
		{
			std::cout<<"registerFactory() - ERROR! A type of id: "<<cmdType<<" already exists!"<<std::endl;
		}

		getFactoryMap()[cmdType] = fnIn;
		std::cout<<"Registered new factory type with id: "<<cmdType<<std::endl;
	}

	std::unique_ptr<BaseCommandPacket> CommandPacketMgr::getCommandPacket(const std::string& rawData)
	{
		std::string err;
		msgpack11::MsgPack base_msgpack = msgpack11::MsgPack::parse(rawData, err);
		
		if (!err.empty())
		{
			std::cerr << "SpotClient::getCommandPacket() - MsgPack parsing error: " << err << std::endl;
			return nullptr;
		}

		msgpack11::MsgPack::array const& elements = base_msgpack.array_items();
		if (elements.size() < 1)
		{
			std::cerr << "SpotClient::getCommandPacket() - INVALID MsgPack array size!" << std::endl;
			return nullptr;
		}

		int cType = elements.front().int32_value();
		std::unique_ptr<BaseCommandPacket> retPacket;
		auto& cFactoryMap = getFactoryMap();
		auto findFactory = cFactoryMap.find(cType);
		if(findFactory != cFactoryMap.end())
		{
			auto& createFunc = findFactory->second;
			retPacket = createFunc();
		}
		else {
			std::cout<<"SpotClient::getCommandPacket() - ERROR! Invalid factory id: "<< cType << std::endl;
		}

		retPacket->parseDataArray(elements);
		return std::move(retPacket);
	}
} // end of namespace SpotClient