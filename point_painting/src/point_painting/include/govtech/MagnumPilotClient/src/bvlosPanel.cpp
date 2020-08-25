#include "bvlosPanel.h"
#include "panelFontRender.hpp"
#include <appCore/imgHolder.h>
#include <appCore/sceneManager.h>
#include <dosCore/wsRobotClient.h>
#include <dosAutonomy/followControl.h>
#include <imguiLib/imgui.h>
#include <Magnum/GL/Renderer.h>
#include <opencv2/opencv.hpp>
#include <bvlosGUI.h>
#include <deepSort.h>

#if defined(_WIN32)
#include <Windows.h>
#include <joystickapi.h>
// BE SURE TO INCLUDE winmm.lib IN ORDER TO USE THE JOYSTICK API
#pragma comment(lib,"winmm.lib")
#endif

namespace DosAutonomy
{

bvlosPanel::bvlosPanel()
	: CanvasPanel()
{
}

bvlosPanel::~bvlosPanel()
{
	sClient->shutdown();
}

void bvlosPanel::init()
{
	loadConfigJSON();
	createGUI();
	createCanvasScene();
	createSpotCamScene();
	createDeepSort();
	spawnThreadedRobotClient();
	spawnThreadedPluginClients();
	setupPluginScenes();
}

void bvlosPanel::canvasTick(float)
{
	auto cScene = SceneManager::instance().getActiveScene();
	if (cScene == nullptr)
	{
		return;
	}

	updateCameraStream();

	updateJoystick();
	updateAutomControls();
	updateStatusStream();
	//updateOdomStream();

	flushCamTextures();
	//testCamUpdate();

	SceneManager::instance().drawActiveScene2D();
}

void bvlosPanel::imguiTick(float)
{
	if (mGUI)
	{
		mGUI->update();
	}
}

/*
void bvlosPanel::pktMsgFn(DosClient::MsgStrCommandPacket& msgIn)
{
	
}
*/

void bvlosPanel::pktImgListFn(DosClient::ImageListCommandPacket& msgIn)
{
	std::lock_guard<std::mutex> scopeLock(mCamLock);
	if (msgIn.getCompressionType() == DosClient::ROBOT_IMG_COMPRESSION_JPG) {

		if (mCameraViewMode == CAMERA_VIEWMODE::DEFAULT) {
			for (auto& cObj : mCamObjects)
			{
				cObj->clearPixels({ 0, 0, 0, 255 });
			}

			for (auto& cImgBytes : msgIn.getImages())
			{
				for (auto& cObj : mCamObjects)
				{
					if (cObj->name() == cImgBytes.label)
					{
						processCamTexture(cImgBytes.bytes, *cObj);
					}
				}
			}

			for (auto& cObj : mCamObjects)
			{
				cObj->flagDirty();
			}
		}
		else if (mCameraViewMode == CAMERA_VIEWMODE::SPOTCAM)
		{
			mSpotCamObj->clearPixels({ 0, 0, 0, 255 });
			if (!msgIn.getImages().empty())
			{
				processCamTexture(msgIn.getImages().front().bytes, *mSpotCamObj);
				mSpotCamObj->flagDirty();
			}

		}

		msgIn.getImages().clear();		
	}

	mUpdateCameraStream = true;
}

void bvlosPanel::pktStatusFn(DosClient::StatusCommandPacket& msgIn)
{
	auto cJSON = msgIn.getStatusJSON();
	if (cJSON.count("battery_percentage") > 0)
	{
		mPowerLeft = cJSON["battery_percentage"].get<float>();
		mPowerLeftStr = cJSON["battery_timeleft"].get<std::string>();
	}

	if (cJSON.count("cameraQuality") > 0)
	{
		mCameraQuality = cJSON["cameraQuality"].get<int>();
	}

	if (cJSON.count("controlStateRecord") > 0)
	{
		mControlStateRecord = cJSON["controlStateRecord"].get<int>() > 0 ? true : false;
	}
	else {
		mControlStateRecord = false;
	}

	if (cJSON.count("spotMode") > 0)
	{
		mSpotMode = cJSON["spotMode"].get<int>();
	}

	if (cJSON.count("logMsg") > 0)
	{
		mGUI->addLogText(cJSON["logMsg"].get<std::string>());
	}

	if (cJSON.count("sCR") > 0)
	{
		mSpotCamRecord = cJSON["sCR"].get<int>();
	}

	if (cJSON.count("imu") > 0)
	{
		mOdomPublish = cJSON["imu"].get<int>() > 0 ? true : false;
	}
}

void bvlosPanel::pktXformFn(DosClient::XformCommandPacket& msgIn)
{
	if (msgIn.subType() == 0)
	{
		mRobotFrameSet = true;
		// Root world frame with position and quaternion
		mRobotPos = Magnum::Vector3(msgIn.pX(), msgIn.pY(), msgIn.pZ());
		mRobotQuat = Magnum::Quaternion({ msgIn.qX(), msgIn.qY(), msgIn.qZ() }, msgIn.qW());
		mRobotLVel = Magnum::Vector3(msgIn.lVelX(), msgIn.lVelY(), msgIn.lVelZ());
		mRobotAVel = Magnum::Vector3(msgIn.aVelX(), msgIn.aVelY(), msgIn.aVelZ());
	}
}

void bvlosPanel::pktSpotGraphNavFn(DosClient::SpotGraphNavCommandPacket& msgIn)
{
	nlohmann::json cJson = msgIn.getStatusJSON();
	if (cJson.count("files") > 0)
	{
		mSpotGraphNavFiles.clear();
		auto& filesObj = cJson["files"];
		for (auto cIter = filesObj.begin(); cIter != filesObj.end(); ++cIter)
		{
			mSpotGraphNavFiles.push_back(*cIter);
		}
	}
}

void bvlosPanel::setCameraFPS(float valIn)
{
	mCameraFPS = std::max(std::min(15.0f, valIn), 1.0f);
}

void bvlosPanel::setCameraFeed(CAMERA_FEED feed_in)
{
	mCamFeed = feed_in;
}

int bvlosPanel::getCameraFeedIdx() const
{
	return static_cast<int>(mCamFeed);
}

void bvlosPanel::reconnectServer()
{
	spawnThreadedRobotClient();
	mUpdateCameraStream = true;
}

void bvlosPanel::spawnThreadedRobotClient()
{
	if (sClient)
	{
		sClient->shutdown();
		sClient.reset();
	}

	mServerAddress = "ws://localhost";
	mServerPort = 8765;
	if ((mRobotConfigJSON.count("serverAddress") > 0) && (mRobotConfigJSON.count("serverPort") > 0))
	{
		mServerAddress = mRobotConfigJSON["serverAddress"].get<std::string>();
		mServerPort = mRobotConfigJSON["serverPort"].get<int>();
	}

	sClient = DosClient::WsClient::startClient<DosClient::WsRobotClient>(mServerAddress, mServerPort);

	sClient->registerPacketRecvFn(
		DosClient::COMMAND_MSG_TYPE,
		[this](DosClient::BaseCommandPacket* pktIn) -> void
		{
			if (auto cPkt = dynamic_cast<DosClient::MsgStrCommandPacket*>(pktIn))
			{
				//pktMsgFn(*cPkt);
			}
		}
	);

	sClient->registerPacketRecvFn(
		DosClient::COMMAND_STATUS_TYPE,
		[this](DosClient::BaseCommandPacket* pktIn) -> void
		{
			if (auto cPkt = dynamic_cast<DosClient::StatusCommandPacket*>(pktIn))
			{
				pktStatusFn(*cPkt);
			}
		}
	);

	sClient->registerPacketRecvFn(
		DosClient::COMMAND_IMG_LIST_TYPE,
		[this](DosClient::BaseCommandPacket* pktIn) -> void
		{
			if (auto cPkt = dynamic_cast<DosClient::ImageListCommandPacket*>(pktIn))
			{
				pktImgListFn(*cPkt);
			}
		}
	);

	sClient->registerPacketRecvFn(
		DosClient::COMMAND_XFORM_TYPE,
		[this](DosClient::BaseCommandPacket* pktIn) -> void
		{
			if (auto cPkt = dynamic_cast<DosClient::XformCommandPacket*>(pktIn))
			{
				pktXformFn(*cPkt);
			}
		}
	);

	sClient->registerPacketRecvFn(
		DosClient::COMMAND_SPOT_NAVGRAPH_TYPE,
		[this](DosClient::BaseCommandPacket* pktIn) -> void
		{
			if (auto cPkt = dynamic_cast<DosClient::SpotGraphNavCommandPacket*>(pktIn))
			{
				pktSpotGraphNavFn(*cPkt);
			}
		}
	);
}

void bvlosPanel::spawnThreadedPluginClients()
{
	// This assumes that there is a spotPluginServices.json located in the ./Data folder
	std::string fileStr = CanvasPanelManager::fileReadAsString("./Data/spotPluginServices.json");
	if (fileStr.empty()) { return; }
	mPluginManager = std::make_unique<DosClient::PluginManager>(fileStr);
}

void bvlosPanel::createFreeTextRender()
{
	auto& canvasManager = DosAutonomy::CanvasPanelManager::instance();
	const auto& viewPortSize = canvasManager.getWinSize();
	mFontRender = std::make_unique<panelFontRender>(viewPortSize.x(), viewPortSize.y());
}

void bvlosPanel::loadConfigJSON()
{
	std::string fileStr = CanvasPanelManager::fileReadAsString("./Data/robotConfig.json");
	if (!fileStr.empty()) {
		mRobotConfigJSON = nlohmann::json::parse(fileStr);
	}
	else {
		std::cout << "ERROR! No ./Data/robotConfig.json present." << std::endl;
		exit(1);
	}
}

void bvlosPanel::createCanvasScene()
{
	{
		// BVLOS Scene for 5 cameras of Spot
		auto bvlosSceneNode = SceneManager::instance().addScene("bvlosScene");

		// Really the Left Image ( it is flipped since we are facing the dog )
		float frontWidth = 16.0f;
		Magnum::Vector2 frontCamSize(frontWidth, frontWidth * 0.75f);
		auto cImg = ImgHolder::createImageObject(
			"frontright_fisheye_image", 640, 480, Magnum::Vector2(-11.5f, 5.0f), frontCamSize, *bvlosSceneNode);
		mCamObjects.push_back(cImg);

		// Really the Right Image ( it is flipped since we are facing the dog )
		cImg = ImgHolder::createImageObject(
			"frontleft_fisheye_image", 640, 480, Magnum::Vector2(4.5f, 5.0f), frontCamSize, *bvlosSceneNode);
		mCamObjects.push_back(cImg);

		// Side and Back Cameras
		float otherWidth = 10.0f;
		Magnum::Vector2 otherCamSize(otherWidth, otherWidth * 0.75f);
		cImg = ImgHolder::createImageObject(
			"left_fisheye_image", 640, 480, Magnum::Vector2(-13.5f, -5.0f), otherCamSize, *bvlosSceneNode);
		mCamObjects.push_back(cImg);

		cImg = ImgHolder::createImageObject(
			"back_fisheye_image", 640, 480, Magnum::Vector2(-3.0f, -5.0f), otherCamSize, *bvlosSceneNode);
		mCamObjects.push_back(cImg);

		cImg = ImgHolder::createImageObject(
			"right_fisheye_image", 640, 480, Magnum::Vector2(7.5f, -5.0f), otherCamSize, *bvlosSceneNode);
		mCamObjects.push_back(cImg);
	}
	SceneManager::instance().setActiveSceneName("bvlosScene");
}

void bvlosPanel::createSpotCamScene()
{
	mSpotCamSceneNode = SceneManager::instance().addScene("spotCamScene");
	mSpotCamObj = ImgHolder::createImageObject(
		"SpotCam", mSpotCamPixelSize.x(), mSpotCamPixelSize.y(), Magnum::Vector2(-4.0f, 0.0f), mSpotCamSize, *mSpotCamSceneNode);
}

void bvlosPanel::createGUI()
{
	mGUI = std::make_unique<bvlosGUI>(*this);
	if (mRobotConfigJSON.count("simpleUI") > 0)
	{
		bool isSimple = mRobotConfigJSON["simpleUI"].get<bool>();
		mGUI->setUIMode(isSimple ? bvlosGUI::UI_MODE::SIMPLE : bvlosGUI::UI_MODE::NORMAL);
	}
}

bool bvlosPanel::createDeepSort()
{
	if ((mRobotConfigJSON.count("videoModelPath") == 0) || (mRobotConfigJSON.count("featuresModelPath") == 0))
	{
		std::cout << "No videoModelPath AND/OR featuresModelPath defined! Cannot initialize Human Tracker." << std::endl;
		return false;
	}

	std::string vidModelPath = mRobotConfigJSON["videoModelPath"].get<std::string>();
	std::string featuresModelPath = mRobotConfigJSON["featuresModelPath"].get<std::string>();
	mInferClient = std::make_unique<DosDNN::InferClient>(vidModelPath.c_str(), true, mOnnxEnv);
	mFeatureClient = std::make_unique<DosDNN::FeatureClient>(featuresModelPath.c_str(), true, mOnnxEnv);
	std::vector<DosDNN::InferResult> myResults;
	mDSManager = std::make_unique<DosDNN::DeepTrackManager>(mDSLambda);
	mFollowControl = std::make_unique<DosClient::FollowControl>();

	return true;
}

void bvlosPanel::processDeepSortImg(cv::Mat& imgIn)
{
	auto drawTrackResults = [this](cv::Mat& drawImg, const cv::Rect& cROI) -> void
	{
		std::array<cv::Scalar, 10> colorPalette{
			cv::Scalar(0, 0, 255),
			cv::Scalar(0, 200, 0),
			cv::Scalar(255, 0, 0),
			cv::Scalar(0, 255, 120),
			cv::Scalar(255, 0, 255),
			cv::Scalar(255, 155, 0),
			cv::Scalar(155, 0, 55),
			cv::Scalar(0, 180, 55),
			cv::Scalar(140, 150, 255),
			cv::Scalar(120, 80, 75)
		};

		int i = 0;
		for (auto& cResult : mDSManager->tracks())
		{
			// Jiayi: Note we do not render the track if it is smaller than certain size cutoffs OR there was no matching bbox for 
			// this current inference step ( aka the age is > 0. When it is assigned the age gets reset back to 0 )
			constexpr float sizeCutoff = 0.1f;
			if ((cResult.width() < sizeCutoff) || (cResult.height() < sizeCutoff) || (cResult.age() > 0))
			{
				continue;
			}

			cv::Point2i sPoint(cResult.x1(), cResult.y1());
			sPoint.x += cROI.x;
			sPoint.y += cROI.y;

			cv::Scalar panelColor = colorPalette[cResult.id() % colorPalette.size()];
			cv::rectangle(
				drawImg,
				cv::Rect2i(sPoint.x, sPoint.y, std::max(int(cResult.width()), 50), 10),
				panelColor, -1, 8, 0);

			cv::rectangle(
				drawImg,
				cv::Rect2i(sPoint.x, sPoint.y, int(cResult.width()), int(cResult.height())),
				panelColor, 3, 8, 0);

			if (cResult.id() == mDSTrackSelectedID)
			{
				static float fadeT = 0.0f;
				fadeT += 0.01f;
				float fadeFrac = (std::sin(fadeT * 40.0f) + 1.0f) * 0.5f;
				auto interpVal = [](float val1, float val2, float frac) -> float
				{
					return ((1.0f - frac) * val1) + (frac * val2);
				};
				cv::Scalar trackColor;
				trackColor[0] = interpVal(panelColor[0], 255.0f, fadeFrac);
				trackColor[1] = interpVal(panelColor[1], 0.0f, fadeFrac);
				trackColor[2] = interpVal(panelColor[2], 0.0f, fadeFrac);

				cv::putText(
					drawImg,
					"Tracking...",
					cv::Point(10, 15),
					cv::FONT_HERSHEY_TRIPLEX,
					0.5,
					cv::Scalar(255 * fadeFrac, 255 * fadeFrac, 0),
					1);

				cv::rectangle(
					drawImg,
					cv::Rect2i(sPoint.x + 10 * fadeFrac, sPoint.y + 10 * fadeFrac, int(cResult.width()) - 20 * fadeFrac, int(cResult.height()) - 20 * fadeFrac),
					trackColor, 2, 8, 0);
			}

			std::string pStr = "OBJ: " + std::to_string(cResult.id());

			cv::putText(
				drawImg,
				pStr,
				cv::Point(sPoint.x, sPoint.y + 10),
				cv::FONT_HERSHEY_TRIPLEX,
				0.35,
				cv::Scalar(10, 10, 10),
				1);
			cv::putText(
				drawImg,
				pStr,
				cv::Point(sPoint.x, sPoint.y + 11),
				cv::FONT_HERSHEY_TRIPLEX,
				0.35,
				cv::Scalar(255, 255, 255),
				1);
			i++;
		}
	};

	// Input SpotCam is 960x480 pixels
	// However, we only want to track a cropped portion of the image
	cv::Mat croppedImg = imgIn(mDSCropROI);
	mDSManager->runDeepSORT(
		croppedImg, 
		mDSInferSize,
		*mInferClient, 
		*mFeatureClient, 
		false);
	drawTrackResults(imgIn, mDSCropROI);
}

void bvlosPanel::testCamUpdate()
{
	auto cScene = SceneManager::instance().getActiveScene();
	if (cScene == nullptr)
	{
		return;
	}

	auto iNode = SceneManager::instance().findDrawHolderWithName<ImgHolder>(*cScene, "frontright_fisheye_image");
	if (iNode)
	{
		static int cnter = 0;
		std::function<void(Magnum::Color4ub&, int, int, int, int)> paintFn =
			[this](Magnum::Color4ub& pixel, int x, int y, int width, int height) -> void
		{
			pixel.r() = uint8_t(float(x) / float(width) * 255.0f);
			pixel.g() = uint8_t(float(y) / float(height) * 255.0f);
			pixel.b() = uint8_t(cnter % 256);
			pixel.a() = 255;
		};
		iNode->foreachImgPixel(paintFn);
		cnter += 10;
	}
	iNode->flagDirty();
}

void bvlosPanel::processCamTexture(std::vector<uint8_t>& bytesIn, ImgHolder& camIn)
{
	cv::Mat camMat = cv::imdecode(bytesIn, cv::IMREAD_COLOR);
	float angle = 0;
	float tx = 0;
	float ty = 0;
	int patchSide = -1;
	if (camIn.name() == "frontleft_fisheye_image")
	{
		angle = -90.0f;
		patchSide = 0;
		tx = -90.0f;
		ty = 0.0f;
	}
	else if (camIn.name() == "frontright_fisheye_image")
	{
		angle = -90.0f;
		patchSide = 1;
		tx = 90.0f;
		ty = 2.0f;
	}
	else if (camIn.name() == "right_fisheye_image")
	{
		angle = -180.0f;
	}

	size_t iWidth = camIn.getImg().size().x();
	size_t iHeight = camIn.getImg().size().y();

	auto adjustCVImg = [this, &iWidth, &iHeight](float angle, float tx, float ty, cv::Mat& imgIn) -> cv::Mat
	{
		// get rotation matrix for rotating the image around its center in pixel coordinates
		cv::Point2f center((imgIn.cols - 1) / 2.0f, (imgIn.rows - 1) / 2.0f);
		cv::Mat xform = cv::getRotationMatrix2D(center, angle, 1.0f);
		// determine bounding rectangle, center not relevant
		cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), imgIn.size(), angle).boundingRect2f();
		// adjust transformation matrix
		xform.at<double>(0, 2) += bbox.width / 2.0 - imgIn.cols / 2.0;
		xform.at<double>(1, 2) += bbox.height / 2.0 - imgIn.rows / 2.0;

		xform.at<double>(0, 2) += tx;
		xform.at<double>(1, 2) += ty;

		cv::Mat dst;
		cv::warpAffine(imgIn, dst, xform, bbox.size());

		cv::Mat outImg;
		cv::Size inputLayerSize(iWidth, iHeight);
		cv::resize(dst, outImg, inputLayerSize);
		return outImg;
	};

	if (std::abs(angle) > 0)
	{
		camMat = adjustCVImg(angle, tx, ty, camMat);
	}

	if (mDeepSortEnabled)
	{
		processDeepSortImg(camMat);
	}

	camIn.setWithCVMatPixels(camMat);
}

void bvlosPanel::flushCamTextures()
{
	std::lock_guard<std::mutex> scopeLock(mCamLock);
	for (auto& iImg : mCamObjects)
	{
		iImg->updateTexture();
	}

	mSpotCamObj->updateTexture();

	auto cPlugin = mPluginManager->getClientWithLayerStr(SceneManager::instance().activeSceneName());
	if (cPlugin)
	{
		auto cIter = mPluginCamObjects.find(cPlugin->mCamLayerStr);
		if (cIter != mPluginCamObjects.end())
		{
			auto& imgObjects = cIter->second;
			for (auto& cImg : imgObjects)
			{
				cImg->updateTexture();
			}
		}
	}
}

std::vector<std::string> bvlosPanel::getCameraFeedStrings() const
{
	auto realCamFeed = mCamFeed;
	if (realCamFeed == CAMERA_FEED::FRONT)
	{
		if (mJoystickPos.y() > 0)
		{
			realCamFeed = CAMERA_FEED::FRONT_AND_REAR;
		}
		else {
			if (mJoystickPos.x() < -0.05f)
			{
				return {
					"frontright_fisheye_image", "frontleft_fisheye_image", "left_fisheye_image"
				};
			}
			else if (mJoystickPos.x() > 0.05f)
			{
				return {
					"frontright_fisheye_image", "frontleft_fisheye_image", "right_fisheye_image"
				};
			}
		}
	}

	if (realCamFeed == CAMERA_FEED::ALL)
	{
		return {
			"back_fisheye_image", "frontleft_fisheye_image", "frontright_fisheye_image",
				"left_fisheye_image", "right_fisheye_image"
		};
	}
	else if (realCamFeed == CAMERA_FEED::FRONT)
	{
		return {
			"frontright_fisheye_image", "frontleft_fisheye_image"
		};
	}
	else if (realCamFeed == CAMERA_FEED::FRONT_AND_REAR)
	{
		return {
			"frontright_fisheye_image", "frontleft_fisheye_image", "back_fisheye_image"
		};
	}
	else if (realCamFeed == CAMERA_FEED::SIDE)
	{
		return {
			"left_fisheye_image", "right_fisheye_image"
		};
	}
	else if (realCamFeed == CAMERA_FEED::REAR)
	{
		return {
			"back_fisheye_image"
		};
	}

	return {};
}

void bvlosPanel::setupPluginScenes()
{
	if (!mPluginManager)
	{
		return;
	}

	for (auto& cPair : mPluginManager->getClients())
	{
		std::shared_ptr<DosClient::PluginClient> cPlugin = cPair.second;
		cPlugin->mCreateCamFn =
			[this, cPlugin](int posX, int posY, int scaleX, int scaleY, const std::string& name, const std::string& descrip)
		{
			auto cSceneNode = SceneManager::instance().addScene(cPlugin->mCamLayerStr);
			auto cImgHolder = ImgHolder::createImageObject(
				name, 640, 480, Magnum::Vector2(posX, posY), Magnum::Vector2(scaleX, scaleY), *cSceneNode);
			mPluginCamObjects[cPlugin->mCamLayerStr].push_back(cImgHolder);
		};
		
		mPluginCamObjects[cPlugin->mCamLayerStr] = {};
		cPlugin->createCameras();

		cPlugin->mStreamFn = [this](DosClient::PluginClient* pClient)
		{
			if (pClient->mCamLayerStr != DosAutonomy::SceneManager::instance().activeSceneName())
			{
				return;
			}
			pluginCamStreamCB(*pClient);
		};
	}
}

void bvlosPanel::pluginCamStreamCB(DosClient::PluginClient& pClient)
{
	auto cIter = mPluginCamObjects.find(pClient.mCamLayerStr);
	if (cIter == mPluginCamObjects.end())
	{
		return;
	}

	auto& imgObjects = cIter->second;
	for (auto& cImg : imgObjects)
	{
		processCamTexture(pClient.mCamBytes, *cImg);
		cImg->flagDirty();
	}
	mUpdateCameraStream = true;
}

Magnum::Vector3 bvlosPanel::getNormalizedJoystickPos(unsigned int jStick, int minVal, int maxVal, float minCutoffToZero) const
{
#if defined(_WIN32)
	JOYINFOEX  joyinfoex;
	joyinfoex.dwSize = sizeof(joyinfoex);
	joyinfoex.dwFlags = JOY_RETURNALL;
	MMRESULT bDev1Attached = false;
	bDev1Attached = joyGetPosEx(jStick, &joyinfoex);
	if ((bDev1Attached == JOYERR_PARMS) || (bDev1Attached == JOYERR_NOCANDO) || (bDev1Attached == JOYERR_UNPLUGGED))
	{
		return { 0.0f, 0.0f, 0.0f };
	}

	float midVal = float(minVal + maxVal) * 0.5f;
	float rangeVal = float(maxVal - minVal) * 0.5f;
	float normX = (float(joyinfoex.dwXpos) - midVal) / rangeVal;
	float normY = (float(joyinfoex.dwYpos) - midVal) / rangeVal;
	float normZ = 1.0f - (float(joyinfoex.dwZpos) / 65535.0f);

	if (std::abs(normX) <= minCutoffToZero)
	{
		normX = 0;
	}

	if (std::abs(normY) <= minCutoffToZero)
	{
		normY = 0;
	}

	//_printToDebug("Joystick [" + std::to_string(normX) + ", " + std::to_string(normY) + "]");
	return { normX, normY, normZ };
#else
	return { 0, 0 };
#endif
}

Magnum::Vector3 bvlosPanel::getOrientationJoystickPos(unsigned int jStick, int minVal, int maxVal, float minCutoffToZero) const
{
#if defined(_WIN32)
	JOYINFOEX  joyinfoex;
	joyinfoex.dwSize = sizeof(joyinfoex);
	joyinfoex.dwFlags = JOY_RETURNALL;
	MMRESULT bDev1Attached = false;
	bDev1Attached = joyGetPosEx(jStick, &joyinfoex);
	if ((bDev1Attached == JOYERR_PARMS) || (bDev1Attached == JOYERR_NOCANDO) || (bDev1Attached == JOYERR_UNPLUGGED))
	{
		return { 0,0,0 };
	}

	float midVal = float(minVal + maxVal) * 0.5f;
	float rangeVal = float(maxVal - minVal) * 0.5f;
	float roll = (float(joyinfoex.dwXpos) - midVal) / rangeVal;
	float pitch = (float(joyinfoex.dwYpos) - midVal) / rangeVal;
	float yaw = (float(joyinfoex.dwRpos) - midVal) / rangeVal;

	if (std::abs(roll) <= minCutoffToZero)
	{
		roll = 0;
	}

	if (std::abs(pitch) <= minCutoffToZero)
	{
		pitch = 0;
	}

	if (std::abs(yaw) <= minCutoffToZero)
	{
		yaw = 0;
	}

	return { roll, pitch, yaw };
#else
	return { 0, 0 };
#endif
}

void bvlosPanel::evtKeyDown(const std::string& str)
{
	char c = str.at(0);
	if (c == '-' || c == '=') {
		switch (c) {
		case '-':
			mConstantVelMag -= 0.1f;
			break;
		case '=':
			mConstantVelMag += 0.1f;
			break;
		}
		mConstantVelMag = std::min(1.0f, std::max(0.0f, mConstantVelMag));
	}
	else if (c == 'W')
	{
		sClient->moveVelRobot(DosClient::MoveVelCommandPacket::VEL_CMD::MOVE_FORWARD, mMoveSpeed);
	}
	else if (c == 'A')
	{
		sClient->moveVelRobot(DosClient::MoveVelCommandPacket::VEL_CMD::STRAFE_LEFT, mMoveSpeed);
	}
	else if (c == 'S')
	{
		sClient->moveVelRobot(DosClient::MoveVelCommandPacket::VEL_CMD::MOVE_BACKWARD, mMoveSpeed);
	}
	else if (c == 'D')
	{
		sClient->moveVelRobot(DosClient::MoveVelCommandPacket::VEL_CMD::STRAFE_RIGHT, mMoveSpeed);
	}
	else if (c == 'Q')
	{
		sClient->moveVelRobot(DosClient::MoveVelCommandPacket::VEL_CMD::TURN_LEFT, mAngularSpeed);
	}
	else if (c == 'E')
	{
		sClient->moveVelRobot(DosClient::MoveVelCommandPacket::VEL_CMD::TURN_RIGHT, mAngularSpeed);
	}
	else if (c == 'N')
	{
		sClient->commandRobot(DosClient::RobotCommandPacket::ROBOT_CMD::STAND);
	}
	else if (c == 'M')
	{
		sClient->commandRobot(DosClient::RobotCommandPacket::ROBOT_CMD::SIT);
	}
	else if (c == 'J')
	{
		sClient->commandRobot(DosClient::RobotCommandPacket::ROBOT_CMD::SELF_RIGHT);
	}
	else if (c == '[')
	{
		sClient->powerRobot(true);
	}
	else if (c == ']')
	{
		sClient->powerRobot(false);
	}
}

void bvlosPanel::onMouseDown(int btn, const Magnum::Vector2i& posIn)
{
	if (mDeepSortEnabled)
	{
		selectHumanTrack(posIn.x(), posIn.y());
	}
}

void bvlosPanel::takeRobot()
{
	sClient->takeRobot();
}

void bvlosPanel::changeMotionParams(std::string robotCmdIn)
{
	nlohmann::json sendJson;
	std::unordered_map<int, std::string> hintStrMap;
	hintStrMap[0] = "HINT_AUTO";
	hintStrMap[1] = "HINT_CRAWL";

	sendJson["locomotion_hint"] = hintStrMap[mGaitTypeIdx];
	sendJson["body_height"] = mBodyHeight;
	sendJson["stair_hint"] = mStairHint;
	sendJson["yaw"] = mYaw;
	sendJson["roll"] = mRoll;
	sendJson["pitch"] = mPitch;
	sendJson["obstacle_avoidance"] = mObstacleAvoidance;

	if (robotCmdIn.empty() == false)
	{
		sendJson["robotCmd"] = robotCmdIn;
	}

	sClient->setRobotMotionParams(sendJson);
}

void bvlosPanel::updateRobotSettings()
{
	nlohmann::json sendJson;
	sendJson["cameraQuality"] = mCameraQuality;
	sClient->setRobotSettings(sendJson);
}

void bvlosPanel::selectHumanTrack(int x, int y)
{
	mDSTrackSelectedID = -1;
	if (!mDSManager)
	{
		return;
	}

	const auto& canvasMgr = CanvasPanelManager::instance();
	/* First scale the position from being relative to window size to being
	   relative to framebuffer size as those two can be different on HiDPI
	   systems */
	const Magnum::Vector2i position = Magnum::Vector2i(x,y) * Magnum::Vector2 { canvasMgr.getFrameBufferSize() } / Magnum::Vector2{ canvasMgr.getWinSize() };
	const Magnum::Vector2i fbPosition{ position.x(), Magnum::GL::defaultFramebuffer.viewport().sizeY() - position.y() - 1 };

	auto projXform = mSpotCamSceneNode->_camera2D.projectionMatrix();
	auto oXform = mSpotCamObj->object().absoluteTransformationMatrix();
	auto camViewport = mSpotCamSceneNode->_camera2D.viewport();
	auto camPixelPos1 = oXform * Magnum::Vector3(-0.5f, -0.5f, 1.0f);
	auto camPixelPos2 = oXform * Magnum::Vector3(0.5f, 0.5f, 1.0f);

	auto calcPixPos = [this, &camViewport, &projXform](Magnum::Vector3& srcPos) -> Magnum::Vector3 {
		Magnum::Vector3 retPos = projXform * srcPos;
		retPos.x() /= retPos.z();
		retPos.y() /= retPos.z();
		retPos.x() = (retPos.x() + 1) * 0.5f * camViewport.x();
		retPos.y() = (retPos.y() + 1) * 0.5f * camViewport.y();
		return retPos;
	};

	camPixelPos1 = calcPixPos(camPixelPos1);
	camPixelPos2 = calcPixPos(camPixelPos2);
	auto camPixelSize = camPixelPos2 - camPixelPos1;

	auto relImgPos = position - Magnum::Vector2i(int(camPixelPos1.x()), int(camPixelPos1.y()));
	float realPixelX = float(relImgPos.x()) / float(camPixelSize.x()) * float(mSpotCamPixelSize.x());
	float realPixelY = float(relImgPos.y()) / float(camPixelSize.y()) * float(mSpotCamPixelSize.y());

	if ((realPixelX < 0) 
		|| (realPixelY < 0) 
		|| (realPixelX >= mSpotCamPixelSize.x()) 
		|| (realPixelY >= mSpotCamPixelSize.y()))
	{
		return;
	}

	float cropPixelX = (realPixelX - mDSCropROI.x);
	float cropPixelY = (realPixelY - mDSCropROI.y);
	for (const auto& cResult : mDSManager->tracks())
	{
		if ((cropPixelX >= cResult.x1())
			&& (cropPixelX <= cResult.x2())
			&& (cropPixelY >= cResult.y1())
			&& (cropPixelY <= cResult.y2()))
		{
			mDSTrackSelectedID = cResult.id();
			break;
		}
	}
}

void bvlosPanel::selectHumanMaxAreaTrack()
{
	if (!mDSManager)
	{
		return;
	}

	const auto& allTracks = mDSManager->tracks();
	if (allTracks.empty())
	{
		mDSTrackSelectedID = -1;
		return;
	}

	auto mIter = std::max_element(allTracks.begin(), allTracks.end(),
		[this](const DosDNN::DeepTrack& a, const DosDNN::DeepTrack& b) -> bool
		{
			return a.area() < b.area();
		}
	);
	mDSTrackSelectedID = mIter->id();
}

void bvlosPanel::updateCameraStream()
{
	std::lock_guard<std::mutex> scopeLock(mCamLock);
	auto rateLimitVideoVal = [this]() -> long long
	{
		return static_cast<long long>(1.0f / mCameraFPS * 1000.0f);
	};

	if (mCameraViewMode == CAMERA_VIEWMODE::SPOTCAM)
	{
		// Need to flush SPOTCAM since their encoder will stutter and drop frames
		auto cTime = CanvasPanelManager::getTimeMS();
		auto diffTime = cTime.count() - mCameraStreamTime.count();
		if ((diffTime > (rateLimitVideoVal() * 3)) && (mUpdateCameraStream == false))
		{
			mUpdateCameraStream = true;
		}
	}

	if (mUpdateCameraStream) {
		auto cTime = CanvasPanelManager::getTimeMS();
		auto diffTime = cTime.count() - mCameraStreamTime.count();
		auto cLimitVal = rateLimitVideoVal();
		if (diffTime > cLimitVal) {
			mCameraStreamTime = cTime;
			mUpdateCameraStream = false;

			if (mCameraViewMode == CAMERA_VIEWMODE::DEFAULT) {
				// Default layer 0 ( default device cameras)
				sClient->sendImageStreamToggle(
					1,
					getCameraFeedStrings()
				);
			}
			else if (mCameraViewMode = CAMERA_VIEWMODE::SPOTCAM) {
				sClient->sendImageStreamToggle(
					1,
					{ "spot_cam" }
				);
			}
			else {
				// Pull from plugin cameras
				auto cPlugin = mPluginManager->getClientWithLayerStr(SceneManager::instance().activeSceneName());
				if (cPlugin)
				{
					cPlugin->streamCamerasFrameStep();
				}
			}

			float cFPS = std::min(15.0f, 1000.0f / static_cast<float>(diffTime));
			mStreamQuality = cFPS / 15.0f * 100.0f;
		}
	}
}

void bvlosPanel::updateJoystick()
{
	// Rate limit so we do not throttle the connection
	auto cTime = CanvasPanelManager::getTimeMS();
	auto diffTime = cTime.count() - mJoystickMoveTime.count();
	auto constexpr cLimitVal = 80;
	if (diffTime > cLimitVal) {
		mJoystickMoveTime = cTime;
	}
	else {
		return;
	}

	// Process Joystick
	if (mMovementMode == MOVEMENT_MODE::MOVE) {
		mJoystickPos = getNormalizedJoystickPos(JOYSTICKID1);

		if (std::abs(mJoystickPos.z() - mJoystickPrevZ) > 0.01f)
		{
			mConstantVelMag = mJoystickPos.z();
			mJoystickPrevZ = mJoystickPos.z();
		}

		if (mConstantVelMag > 0.0f)
		{
			if (mJoystickPos.y() > 0.0f)
			{
				mConstantVelMag = 0.0f;
			}
			sClient->freeMoveVelRobot(mConstantVelMag, 0.0f, -mJoystickPos.x(), mMoveSpeed);
		}
		else {
			if ((std::abs(mJoystickPos.x()) > 0) || (std::abs(mJoystickPos.y()) > 0))
			{
				sClient->freeMoveVelRobot(-mJoystickPos.y(), 0.0f, -mJoystickPos.x(), mMoveSpeed);
			}
		}
	}
	else if (mMovementMode == MOVEMENT_MODE::STAND) {
		mJoystickOrientation = getOrientationJoystickPos(JOYSTICKID1);
		if (mJoystickOrientation.length() > 0)
		{
			mRoll = mJoystickOrientation.x();
			mPitch = -mJoystickOrientation.y();
			mYaw = mJoystickOrientation.z();

			// Now map to radians
			float piVal = pi();
			mRoll *= piVal / 4.0f;
			mPitch *= piVal / 4.0f;
			mYaw *= -piVal / 4.0f;
			changeMotionParams("stand");
		}
	}
}

void bvlosPanel::updateAutomControls()
{
	// Rate limit so we do not throttle the connection
	auto cTime = CanvasPanelManager::getTimeMS();
	auto diffTime = cTime.count() - mAutomMoveTime.count();
	auto constexpr cLimitVal = 100;
	if (diffTime > cLimitVal) {
		mAutomMoveTime = cTime;
	}
	else {
		return;
	}

	auto isValid = (mDSManager && mFollowControl && mDeepSortEnabled);
	if (!isValid)
	{
		return;
	}

	if (mAutoTrack)
	{
		selectHumanMaxAreaTrack();
		if (mDSTrackSelectedID != -1)
		{
			mAutoTrack = false;
		}
	}

	auto cTrack = mDSManager->getTrackByID(mDSTrackSelectedID);
	if (cTrack == nullptr)
	{
		return;
	}

	auto followVec = mFollowControl->computeFollowFreeMove(
		glm::vec2(mDSCropROI.width, mDSCropROI.height),
		glm::vec2(cTrack->x1(), cTrack->y1()),
		glm::vec2(cTrack->x2(), cTrack->y2()));
	if (glm::length(followVec) > 0.1f)
	{
		sClient->freeMoveVelRobot(followVec.y, 0.0f, followVec.x, followVec.z);
	}
}

void bvlosPanel::updateStatusStream()
{
	auto cTime = CanvasPanelManager::getTimeMS();
	auto diffTime = cTime.count() - mStatusStreamTime.count();
	constexpr int statusDuration = 2500;
	if (diffTime > statusDuration)
	{
		mStatusStreamTime = cTime;
		sClient->sendRobotStatusStr();
	}
}

void bvlosPanel::updateOdomStream()
{
	auto cTime = CanvasPanelManager::getTimeMS();
	auto diffTime = cTime.count() - mOdomStreamTime.count();
	constexpr int statusDuration = 1000;
	if (diffTime > statusDuration)
	{
		mOdomStreamTime = cTime;
		sClient->sendRobotOdomStr();
	}
}

} // end of namepace DosAutonomy