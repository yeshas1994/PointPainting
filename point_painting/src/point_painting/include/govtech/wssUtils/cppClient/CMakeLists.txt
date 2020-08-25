#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(wssClientCPP)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# Make project require C++14
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
endif()

find_package(OpenSSL REQUIRED)

# Source files
set(TARGET_SRC
	main.cpp
	../../dosFramework/dosCore/wsClient.h
	../../dosFramework/dosCore/wsClient.cpp
	../../msgpack11/msgpack11.cpp
	../../msgpack11/msgpack11.hpp
	../../dosFramework/dosCore/commandPacket.h
	../../dosFramework/dosCore/commandPacket.cpp
	../../dosFramework/dosCore/commandPacketParse.h
	../../dosFramework/dosCore/commandPacketParse.cpp
)

# Add preprocessor definitions
add_definitions(-DASIO_STANDALONE -D_WEBSOCKETPP_CPP11_TYPE_TRAITS_ -DDOS_TLS_ENABLE)

# getting-started
add_executable(wssClientCPP ${TARGET_SRC})

include_directories(
	thermalCamPlugin
	${ADDITIONAL_INCLUDE_DIRS}
	"../../json/include/"
    "../../websocketpp/"
	"../../asio-1.12.2/include/"
	"../../msgpack11/"
	"../../dosFramework/"
	"../../stb/"
)

target_link_libraries(
    wssClientCPP 
    OpenSSL::SSL
    OpenSSL::Crypto
)

set_target_properties (wssClientCPP PROPERTIES FOLDER Examples)