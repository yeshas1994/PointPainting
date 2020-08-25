#pragma once
#include <appCore/canvasPanel.h>
#include <appCore/sceneManager.h>
#include <Magnum/Math/Vector2.h>
#include <Magnum/Math/Vector3.h>
#include <Magnum/Math/Quaternion.h>
#include <Magnum/Magnum.h>
#include <dosCore/wsRobotClient.h>
#include <pluginClients/pluginServices.h>
#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>

namespace DosDNN
{
	class DeepTrackManager;
	class InferClient;
	class FeatureClient;
}

namespace DosClient
{
	class FollowControl;
}

namespace DosAutonomy
{

class ImgHolder;
class bvlosGUI;
class panelFontRender;

class bvlosPanel : public CanvasPanel
{
public:
	enum FOLLOW_MODE
	{
		NONE,
		CLOSEST_HUMAN,
		TRACK_HUMAN,
		TRACK_HUMAN2,
		SIMPLE_PATH
	};

	enum CAMERA_FEED
	{
		ALL,
		FRONT,
		FRONT_AND_REAR,
		SIDE,
		REAR
	};

	enum MOVEMENT_MODE
	{
		MOVE,
		STAND
	};

	enum CAMERA_VIEWMODE
	{
		DEFAULT,
		PLUGIN,
		SPOTCAM
	};

public:
	bvlosPanel();
	virtual ~bvlosPanel();

	// Event overrides
	virtual void init() override;
	virtual void canvasTick(float delta_time) override;
	virtual void imguiTick(float delta_time) override;

	// Input overrides
	virtual void evtKeyDown(const std::string& str) override;
	virtual void onMouseDown(int btn, const Magnum::Vector2i& posIn);

	void takeRobot();
	void changeMotionParams(std::string robotCmdIn = "");
	void updateRobotSettings();

	void selectHumanTrack(int x, int y);
	void selectHumanMaxAreaTrack();

	// Received CommandPacket Processing
	//void pktMsgFn(DosClient::MsgStrCommandPacket& msgIn);
	void pktImgListFn(DosClient::ImageListCommandPacket& msgIn);
	void pktStatusFn(DosClient::StatusCommandPacket& msgIn);
	void pktXformFn(DosClient::XformCommandPacket& msgIn);
	void pktSpotGraphNavFn(DosClient::SpotGraphNavCommandPacket& msgIn);

	void setCameraFPS(float valIn);
	float getCameraFPS() const { return mCameraFPS; }
	void setCameraFeed(CAMERA_FEED feed_in);
	int getCameraFeedIdx() const;

	void reconnectServer();

protected:
	void spawnThreadedRobotClient();
	void spawnThreadedPluginClients();

	void createFreeTextRender();
	void loadConfigJSON();
	void createCanvasScene();
	void createSpotCamScene();
	void createGUI();
	bool createDeepSort();
	void processDeepSortImg(cv::Mat& imgIn);
	void testCamUpdate();
	void processCamTexture(std::vector<uint8_t>& bytesIn, ImgHolder& camIn);
	void flushCamTextures();
	std::vector<std::string> getCameraFeedStrings() const;
	float pi() const { return std::atan(1.0f) * 4.0f; } // Compiler should optimize this out in Release Mode

	void setupPluginScenes();
	void pluginCamStreamCB(DosClient::PluginClient& pClient);

	Magnum::Vector3 getNormalizedJoystickPos(unsigned int jStick, int minVal = 0, int maxVal = 65535, float minCutoffToZero = 0.05f) const;
	Magnum::Vector3 getOrientationJoystickPos(unsigned int jStick, int minVal = 0, int maxVal = 65535, float minCutoffToZero = 0.05f) const;


	void updateCameraStream();
	void updateJoystick();
	void updateAutomControls();
	void updateStatusStream();
	void updateOdomStream();

public:
	std::shared_ptr<DosClient::WsRobotClient> sClient;
	std::unique_ptr<DosClient::PluginManager> mPluginManager;
	std::unique_ptr<bvlosGUI> mGUI;
	std::unique_ptr<panelFontRender> mFontRender;

	DosAutonomy::SceneManager::SceneNode* mSpotCamSceneNode = nullptr; // Yup raw pointers, but it's not me, it's the Magnum Engine...
	static constexpr Magnum::Vector2 mSpotCamSize{ 28.0f, 14.0f };
	static constexpr Magnum::Vector2i mSpotCamPixelSize{ 960, 480 };

	std::unique_ptr<DosDNN::InferClient> mInferClient;
	std::unique_ptr<DosDNN::FeatureClient> mFeatureClient;
	std::unique_ptr<DosDNN::DeepTrackManager> mDSManager;
	int mDSTrackSelectedID = -1;
	cv::Rect mDSCropROI = cv::Rect(280, 0, 396, 267); // OpenCV does not allow constexpr?
	cv::Size2i mDSInferSize = cv::Size2i(416, 288); // OpenCV does not allow constexpr?
	static constexpr float mDSLambda = 0.5f;
	std::unique_ptr<DosClient::FollowControl> mFollowControl;

	Ort::Env mOnnxEnv = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "test");

	float mMoveSpeed = 1.0f;
	float mAngularSpeed = 0.6f;
	Magnum::Vector3 mJoystickPos = { 0, 0, 0 };
	float mJoystickPrevZ = 0.0;
	Magnum::Vector3 mJoystickOrientation = { 0, 0, 0 };
	std::unordered_map<char, int> mKeyCommandState;

	int mGaitTypeIdx = 0;
	float mBodyHeight = 0.0f;
	bool mStairHint = false;
	float mYaw = 0.0f, mRoll = 0.0f, mPitch = 0.0f;
	bool mObstacleAvoidance = true;
	float mConstantVelMag = 0.0f;

	std::mutex mCamLock;
	bool mCamerasActive = false;
	float mCameraFPS = 15.0f;
	CAMERA_FEED mCamFeed = CAMERA_FEED::FRONT;
	CAMERA_VIEWMODE mCameraViewMode = CAMERA_VIEWMODE::DEFAULT;
	std::chrono::milliseconds mCameraStreamTime = CanvasPanelManager::getTimeMS();
	std::chrono::milliseconds mStatusStreamTime = CanvasPanelManager::getTimeMS();
	std::chrono::milliseconds mOdomStreamTime = CanvasPanelManager::getTimeMS();
	std::chrono::milliseconds mJoystickMoveTime = CanvasPanelManager::getTimeMS();
	std::chrono::milliseconds mAutomMoveTime = CanvasPanelManager::getTimeMS();
	bool mUpdateCameraStream = true;
	float mPowerLeft = 1.0f;
	std::string mPowerLeftStr = "None";
	float mStreamQuality = 100.0f;
	int mCameraQuality = 10;
	int mSpotCamRecord = 0;
	bool mControlStateRecord = false;
	bool mOdomPublish = false;
	MOVEMENT_MODE mMovementMode = MOVEMENT_MODE::MOVE;
	Magnum::Vector3 mRobotPos = { 0, 0, 0 };
	Magnum::Quaternion mRobotQuat = Magnum::Quaternion(Magnum::Math::ZeroInit);
	Magnum::Vector3 mRobotLVel = { 0, 0, 0 };
	Magnum::Vector3 mRobotAVel = { 0, 0, 0 };
	bool mRobotFrameSet = false;
	std::vector<std::shared_ptr<ImgHolder>> mCamObjects;
	std::shared_ptr<ImgHolder> mSpotCamObj;
	std::unordered_map<std::string, std::vector<std::shared_ptr<ImgHolder>>> mPluginCamObjects;
	std::vector<std::string> mSpotGraphNavFiles;
	bool mDeepSortEnabled = false;
	bool mAutoTrack = false;

	static constexpr int SPOT_RUN_NORMAL_MODE = 0;
	static constexpr int SPOT_RUN_NAV_RECORD_MODE = 1;
	static constexpr int SPOT_RUN_NAV_PLAY_MODE = 2;
	int mSpotMode = SPOT_RUN_NORMAL_MODE;

	nlohmann::json mRobotConfigJSON;
	std::string mServerAddress;
	int mServerPort = 0;
};

} // end of namepace DosPilotApp