#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(cmdPlugin)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# Make project require C++14
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
endif()

if(UNIX)
    find_package(OpenSSL REQUIRED)
endif()

# Source files
set(TARGET_SRC
	main.cpp
	../dosFramework/dosCore/wsServer.h
	../dosFramework/dosCore/wsServer.cpp
	../msgpack11/msgpack11.cpp
	../msgpack11/msgpack11.hpp
	../dosFramework/dosCore/commandPacket.h
	../dosFramework/dosCore/commandPacket.cpp
	../dosFramework/dosCore/commandPacketParse.h
	../dosFramework/dosCore/commandPacketParse.cpp
)

# Add preprocessor definitions
add_definitions(-DASIO_STANDALONE -D_WEBSOCKETPP_CPP11_TYPE_TRAITS_)

# getting-started
add_executable(cmdPlugin ${TARGET_SRC})

include_directories(
	cmdPlugin
	${ADDITIONAL_INCLUDE_DIRS}
	"../json/include/"
    "../websocketpp/"
	"../asio-1.12.2/include/"
	"../msgpack11/"
	"../glm/"
	"../dosFramework/"
)

if(UNIX)
    target_link_libraries(
        cmdPlugin 
        pthread
        OpenSSL::SSL
    )
endif()