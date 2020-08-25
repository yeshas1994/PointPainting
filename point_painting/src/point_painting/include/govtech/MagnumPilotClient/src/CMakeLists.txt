cmake_minimum_required(VERSION 3.4)


# Add preprocessor definitions
add_definitions(-DASIO_STANDALONE -D_WEBSOCKETPP_CPP11_TYPE_TRAITS_)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})

find_package(Corrade REQUIRED Main)
find_package(Magnum CONFIG REQUIRED 
    GL 
    MeshTools
    Primitives
    SceneGraph
    Shaders
	Text
    Sdl2Application
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

# The following folder will be included
if(UNIX) 
	set(ONNX_FOLDER_NAME "onnxruntime-linux-x64-gpu-1.3.0")
else()
	set(ONNX_FOLDER_NAME "onnxruntime-win-x64-gpu-1.3.0")
endif()

# Source files
set(TARGET_SRC
    MainApp.cpp
	bvlosPanel.h
	bvlosPanel.cpp
	bvlosGUI.h
	bvlosGUI.cpp
	panelFontRender.hpp
)

source_group("mainSourceFiles" FILES ${TARGET_SRC})

corrade_add_resource(MagnumImGuiIntegrationShaders_RCS ../../MagnumAppCore/imguiLib/resources.conf)
corrade_add_resource(APP_RESOURCES ./res/resources.conf)

# ImGui files
set(IMGUI_FILES
    ../../MagnumAppCore/imguiLib/Context.h
    ../../MagnumAppCore/imguiLib/Context.cpp
    ../../MagnumAppCore/imguiLib/Integration.h
    ../../MagnumAppCore/imguiLib/visibility.h
    ../../MagnumAppCore/imguiLib/imgui.h
    ../../MagnumAppCore/imguiLib/imgui.cpp
    ../../MagnumAppCore/imguiLib/imgui_draw.cpp
    ../../MagnumAppCore/imguiLib/imgui_widgets.cpp
)

source_group("imguiFiles" FILES ${IMGUI_FILES})

# Plugin files
set(PLUGINS_SRC
	pluginClients/pluginServices.h
	pluginClients/pluginServices.cpp
	pluginClients/pluginRealsenseClient.h
	pluginClients/pluginRealsenseClient.cpp
	pluginClients/pluginThermalCamClient.h
	pluginClients/pluginThermalCamClient.cpp
	pluginClients/pluginCmdClient.h
	pluginClients/pluginCmdClient.cpp
)

source_group("pluginFiles" FILES ${PLUGINS_SRC})

# DeepSORT files
set(DEEPSORT_FILES
	../../deepSort/deepSort.cpp
	../../deepSort/deepSort.h
	../../deepSort/dnnUtils.hpp
	../../deepSort/dnnUtils.cpp
	../../deepSort/featureExtract.h
	../../deepSort/featureExtract.cpp
	../../deepSort/Hungarian.cpp
	../../deepSort/Hungarian.h
	../../deepSort/imgInfer.cpp
	../../deepSort/imgInfer.h
)

source_group("deepsortFiles" FILES ${DEEPSORT_FILES})

# Msgpack files
set(MSGPACK_FILES
	../../msgpack11/msgpack11.cpp
	../../msgpack11/msgpack11.hpp
)

source_group("msgpackFiles" FILES ${MSGPACK_FILES})

set(APPCORE_SRC
    ../../MagnumAppCore/appCore/Scene.h
    ../../MagnumAppCore/appCore/Camera.h
    ../../MagnumAppCore/appCore/Camera.cpp
    ../../MagnumAppCore/appCore/sceneManager.h
    ../../MagnumAppCore/appCore/sceneManager.cpp
    ../../MagnumAppCore/appCore/imgHolder.cpp
    ../../MagnumAppCore/appCore/imgHolder.h
	../../MagnumAppCore/appCore/canvasPanel.h
	../../MagnumAppCore/appCore/canvasPanel.cpp
)

source_group("appCore" FILES ${APPCORE_SRC})

set(DOSFRAMEWORK_SRC
    ../../dosFramework/dosCore/wsClient.h
    ../../dosFramework/dosCore/wsClient.cpp
	../../dosFramework/dosCore/wsRobotClient.h
	../../dosFramework/dosCore/wsRobotClient.cpp
	../../dosFramework/dosCore/commandPacket.h
	../../dosFramework/dosCore/commandPacket.cpp
	../../dosFramework/dosCore/commandPacketParse.h
	../../dosFramework/dosCore/commandPacketParse.cpp
	../../dosFramework/dosAutonomy/followControl.h
	../../dosFramework/dosAutonomy/followControl.cpp
)
source_group("dosCore" FILES ${DOSFRAMEWORK_SRC})

# The following folder will be included
include_directories(
    "${PROJECT_SOURCE_DIR}/src"
    "../../websocketpp/"
	"../../asio-1.12.2/include/"
	"../../msgpack11/"
	"../../stb/"
	"../../json/single_include/"
	"../../dosFramework/"
	"../../glm/"
	"../../MagnumAppCore"
    "../../deepSort"
    "../../../${ONNX_FOLDER_NAME}/include/"
)

if(UNIX) 
get_filename_component(ONNX_LIB_ABSOLUTE_PATH "../../../${ONNX_FOLDER_NAME}/lib/libonnxruntime.so" ABSOLUTE)
else()
get_filename_component(ONNX_LIB_ABSOLUTE_PATH "../../../${ONNX_FOLDER_NAME}/lib/onnxruntime.lib" ABSOLUTE)
endif()

# Final Link + Executable
add_executable(MagnumPilotApp ${TARGET_SRC} ${APPCORE_SRC} ${IMGUI_FILES} ${DOSFRAMEWORK_SRC} ${MSGPACK_FILES} ${DEEPSORT_FILES} ${PLUGINS_SRC} ${MagnumImGuiIntegrationShaders_RCS} ${APP_RESOURCES})
target_link_libraries(MagnumPilotApp PRIVATE
    Corrade::Main
    Magnum::Application
    Magnum::GL
    Magnum::Magnum
    Magnum::Shaders
    Magnum::SceneGraph
    Magnum::MeshTools
    Magnum::Primitives
	Magnum::Text
    opencv_world
    ${ONNX_LIB_ABSOLUTE_PATH}
)