#include <bvlosGUI.h>
#include <imguiLib/imgui.h>
#include <appCore/sceneManager.h>
#include <bvlosPanel.h>
#include <iomanip>
#include <sstream>

namespace DosAutonomy
{
// Logging UI
class logUI
{
public:
	logUI() = default;

	void clear() { m_Buf.clear(); }

	bool hasText() const { return (m_Buf.size() > 0); }

	void addLog(const char * msgIn)
	{
		m_Buf.appendf(msgIn);
		m_ScrollToBottom = true;
	}

	void draw()
	{
		if (!hasText())
		{
			return;
		}

		ImGui::Begin("Log Messages");
		if (ImGui::Button("Clear")) {
			clear();
		}

		ImGui::SameLine();
		bool copy = ImGui::Button("Copy");
		ImGui::Separator();
		ImGui::BeginChild("scrolling");
		ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0,1));
		if (copy) { ImGui::LogToClipboard(); }

		ImGui::TextUnformatted(m_Buf.begin());
		if (m_ScrollToBottom) {
			ImGui::SetScrollHere(1.0f);
		}

		m_ScrollToBottom = false;
		ImGui::PopStyleVar();
		ImGui::EndChild();
		ImGui::End();
	}

public:
	ImGuiTextBuffer     m_Buf;
	bool                m_ScrollToBottom = false;
};

// bvlosGUI
bvlosGUI::bvlosGUI(bvlosPanel& pPanelIn)
	: pPanel(pPanelIn)
{
	mLogUI = std::make_unique<logUI>();
}

bvlosGUI::~bvlosGUI()
{
}

void bvlosGUI::update()
{
	runUI();
}

void bvlosGUI::addLogText(const std::string& msgIn)
{
	mLogUI->addLog((msgIn + "\n").c_str());
}

void bvlosGUI::runUI()
{
	drawMainUI();
	drawPluginsUI();
	drawLogUI();
}

void bvlosGUI::drawMainUI()
{
	ImGui::Begin("LIMA Controls");
	{
		bool isSimpleMode = (getUIMode() == UI_MODE::SIMPLE);
		if (ImGui::Checkbox("Simple Mode", &isSimpleMode))
		{
			setUIMode(isSimpleMode ? UI_MODE::SIMPLE : UI_MODE::NORMAL);
		}
	}

	drawCameraUI();
	drawPowerUI();
	drawDetectionUI();
	drawMovementUI();

	if (getUIMode() == UI_MODE::NORMAL) {
		drawAdvancedUI();
	}
	drawAutonomyUI();

	drawJoystick();
	drawTrajectory();

	ImGui::End();
}

void bvlosGUI::drawCameraUI()
{
	auto& sceneManager = SceneManager::instance();
	float camFPS = pPanel.getCameraFPS();
	if (getUIMode() == UI_MODE::NORMAL) {
		ImGui::InputFloat("Camera FPS", &camFPS);
		pPanel.setCameraFPS(camFPS);
	}

	// Pick/choose camera layer to show
	std::vector<std::string> cameraTypeNames(2);
	if (pPanel.mPluginManager) {
		auto& cPluginMap = pPanel.mPluginManager->getClients();
		cameraTypeNames = std::vector<std::string>(cameraTypeNames.size() + cPluginMap.size());
	}

	cameraTypeNames[0] = "Default";
	cameraTypeNames[1] = "SpotCam";
	int m = 2;
	int camActiveSelectIdx = (pPanel.mCameraViewMode == bvlosPanel::CAMERA_VIEWMODE::DEFAULT) ? 0 : 1;
	std::vector<const char*> cameraTypeNameStrs(cameraTypeNames.size());
	if (pPanel.mPluginManager) {
		auto& cPluginMap = pPanel.mPluginManager->getClients();
		for (auto& cPair : cPluginMap)
		{
			cameraTypeNames[m] = cPair.first;
			auto& cPlugin = cPair.second;
			if (cPlugin->mCamLayerStr == sceneManager.activeSceneName())
			{
				camActiveSelectIdx = m;
			}
			m++;
		}
		for (size_t i = 0; i < cameraTypeNames.size(); i++)
		{
			cameraTypeNameStrs[i] = cameraTypeNames[i].data();
		}
	}
	else {
		cameraTypeNameStrs[0] = cameraTypeNames[0].data();
	}

	if (getUIMode() == UI_MODE::NORMAL) {
		if (ImGui::Combo("Cameras", &camActiveSelectIdx, cameraTypeNameStrs.data(), (int)cameraTypeNameStrs.size()))
		{
			if (camActiveSelectIdx == 0)
			{
				// Switch back to default bvlos
				sceneManager.setActiveSceneName("bvlosScene");
				pPanel.mCameraViewMode = bvlosPanel::CAMERA_VIEWMODE::DEFAULT;
				pPanel.mUpdateCameraStream = true;
				pPanel.mDeepSortEnabled = false;
			}
			else if (camActiveSelectIdx == 1)
			{
				// Switch to SpotCam
				sceneManager.setActiveSceneName("spotCamScene");
				pPanel.mCameraViewMode = bvlosPanel::CAMERA_VIEWMODE::SPOTCAM;
				pPanel.mUpdateCameraStream = true;
			}
			else {
				// Find plugin cameras
				const auto& camTypeName = cameraTypeNames[camActiveSelectIdx];
				auto cPlugin = pPanel.mPluginManager->getClientWithTypeName(camTypeName);
				if (cPlugin)
				{
					sceneManager.setActiveSceneName(cPlugin->mCamLayerStr);
					pPanel.mCameraViewMode = bvlosPanel::CAMERA_VIEWMODE::PLUGIN;
				}
			}
		}

		if ((camActiveSelectIdx == 1) && (pPanel.mDSManager))
		{
			ImGui::Checkbox("Track Humans", &pPanel.mDeepSortEnabled);
			ImGui::SameLine();
			ImGui::Checkbox("Auto Track", &pPanel.mAutoTrack);
		}
	}

	// Camera feed
	if (sceneManager.activeSceneName() == "bvlosScene") {
		// Default device cameras
		std::array<const char*, 5> cameraTypes = { "All", "Front", "Front_And_Rear", "Side", "Rear" };
		int selectedItem = pPanel.getCameraFeedIdx();
		if (ImGui::Combo("Feed", &selectedItem, cameraTypes.data(), (int)cameraTypes.size()))
		{
			bvlosPanel::CAMERA_FEED setFeed = bvlosPanel::CAMERA_FEED::FRONT;
			if (selectedItem == 0)
			{
				setFeed = bvlosPanel::CAMERA_FEED::ALL;
			}
			else if (selectedItem == 1)
			{
				setFeed = bvlosPanel::CAMERA_FEED::FRONT;
			}
			else if (selectedItem == 2)
			{
				setFeed = bvlosPanel::CAMERA_FEED::FRONT_AND_REAR;
			}
			else if (selectedItem == 3)
			{
				setFeed = bvlosPanel::CAMERA_FEED::SIDE;
			}
			else if (selectedItem == 4)
			{
				setFeed = bvlosPanel::CAMERA_FEED::REAR;
			}
			pPanel.setCameraFeed(setFeed);
		}
	}
	else {
		// Custom plugin cameras
		//auto& cPluginMap = pPanel.mPluginManager->getClients();
	}

	int cCamQuality = getActiveCamQuality();
	if (ImGui::InputInt("Quality", &cCamQuality, 1, 10, ImGuiInputTextFlags_EnterReturnsTrue))
	{
		setActiveCamQuality(cCamQuality);
	}
	
}

void bvlosGUI::drawPowerUI()
{
	ImGui::Separator();
	if (ImGui::Button("Power ON: ["))
	{
		pPanel.sClient->powerRobot(true);
	}
	ImGui::SameLine();
	if (ImGui::Button("Power OFF: ]"))
	{
		pPanel.sClient->powerRobot(false);
	}

	ImGui::SameLine();
	if (ImGui::Button("Restart Server"))
	{
		pPanel.sClient->sendRobotMsgStr("RESTART_SPOT_SERVER");
	}

	ImGui::ProgressBar(pPanel.mPowerLeft, ImVec2(-1, 0), pPanel.mPowerLeftStr.c_str());

	if (getUIMode() == UI_MODE::NORMAL) {
		std::stringstream rStream;
		rStream << "Pos: (" << std::fixed << std::setprecision(1)
			<< pPanel.mRobotPos.x() << ", " << pPanel.mRobotPos.y() << ", " << pPanel.mRobotPos.z() << ")"
			<< " Distance: " << totalTrajectoryDistance() << std::endl;
		ImGui::BulletText(rStream.str().c_str());

		std::stringstream vStream;
		vStream << "LVel: (" << std::fixed << std::setprecision(1)
			<< pPanel.mRobotLVel.x() << ", " << pPanel.mRobotLVel.y() << ", " << pPanel.mRobotLVel.z() << ")"
			<< " AVel: (" <<
			pPanel.mRobotAVel.x() << ", " << pPanel.mRobotAVel.y() << ", " << pPanel.mRobotAVel.z() << ")"
			<< std::endl;
		ImGui::BulletText(vStream.str().c_str());
	}

	std::array<char, 512> connectionStr;
	int connectQuality = static_cast<int>(pPanel.mStreamQuality);
	char connectGoodDesrip[] = "GOOD";
	char connectAvgDesrip[] = "AVERAGE";
	char connectPoorDesrip[] = "POOR";
	char* connectDescrip = nullptr;
	if (connectQuality >= 80) {
		connectDescrip = connectGoodDesrip;
	}
	else if (connectQuality < 80 && connectQuality >= 45) {
		connectDescrip = connectAvgDesrip;
	}
	else {
		connectDescrip = connectPoorDesrip;
	}

	sprintf(connectionStr.data(), "Connection %s (%d)", connectDescrip, static_cast<int>(pPanel.mStreamQuality));
	ImGui::BulletText(connectionStr.data());
	ImGui::SameLine();
	if (ImGui::Button("Reconnect"))
	{
		pPanel.reconnectServer();
	}
}

void bvlosGUI::drawDetectionUI()
{
}

void bvlosGUI::drawMovementUI()
{
	if (ImGui::CollapsingHeader("Movement")) {
		if (ImGui::Button("Stand n")) {
			pPanel.sClient->commandRobot(DosClient::RobotCommandPacket::ROBOT_CMD::STAND);
		}
		ImGui::SameLine();
		if (ImGui::Button("Sit: m")) {
			pPanel.sClient->commandRobot(DosClient::RobotCommandPacket::ROBOT_CMD::SIT);
		}
		ImGui::SameLine();
		if (ImGui::Button("Self-right: j")) {
			pPanel.sClient->commandRobot(DosClient::RobotCommandPacket::ROBOT_CMD::SELF_RIGHT);
		}
		ImGui::SameLine();
		if (ImGui::Checkbox("Stairs", &pPanel.mStairHint)) {
			pPanel.changeMotionParams();
		}

		if (getUIMode() == UI_MODE::NORMAL) {
			drawMotionParamsUI();
		}
		else if (getUIMode() == UI_MODE::SIMPLE) {
			drawMotionParamsSimpleUI();
		}
	}
}

void bvlosGUI::drawMotionParamsUI()
{	
	bool pubOdom = pPanel.mOdomPublish;
	if (ImGui::Checkbox("Odom", &pubOdom))
	{
		if (pubOdom)
		{
			pPanel.sClient->sendRobotMsgStr("odomPublishON");
		}
		else {
			pPanel.sClient->sendRobotMsgStr("odomPublishOFF");
		}
		pPanel.mOdomPublish = pubOdom;
	}

	ImGui::SameLine();
	if (ImGui::Checkbox("Obstacle Avoidance", &pPanel.mObstacleAvoidance)) {
		pPanel.changeMotionParams();
	}

	bool changeMotionParams = false;
	std::array<const char*, 2> gaitTypes = { "Auto", "Crawl" };
	if (ImGui::Combo("Gait", &pPanel.mGaitTypeIdx, gaitTypes.data(), (int)gaitTypes.size()))
	{
		changeMotionParams = true;
	}

	int moveSpeed = int(pPanel.mMoveSpeed * 100.0f);
	if (ImGui::InputInt("Speed", &moveSpeed, 20, 10, ImGuiInputTextFlags_EnterReturnsTrue))
	{
		moveSpeed = std::max(10, std::min(220, moveSpeed));
		pPanel.mMoveSpeed = float(moveSpeed) * 0.01f;
	}

	ImGui::SliderFloat("Cruise Control", &pPanel.mConstantVelMag, 0.0f, 1.0f);

	if (ImGui::InputFloat("Height", &pPanel.mBodyHeight, 0, 0, 1, ImGuiInputTextFlags_EnterReturnsTrue))
	{
		changeMotionParams = true;
	}
	if (changeMotionParams)
	{
		pPanel.changeMotionParams();
	}

	std::array<const char*, 2> movementModes = { "Move", "Stand" };
	int movmentIdx = pPanel.mMovementMode;
	if (ImGui::Combo("Movement Mode", &movmentIdx, movementModes.data(), (int)movementModes.size()))
	{
		pPanel.mMovementMode = (bvlosPanel::MOVEMENT_MODE)movmentIdx;
	}
}

void bvlosGUI::drawMotionParamsSimpleUI()
{
	int moveSpeed = int(pPanel.mMoveSpeed * 100.0f);
	if (ImGui::InputInt("Speed", &moveSpeed, 20, 10, ImGuiInputTextFlags_EnterReturnsTrue))
	{
		moveSpeed = std::max(10, std::min(220, moveSpeed));
		pPanel.mMoveSpeed = float(moveSpeed) * 0.01f;
	}
}

void bvlosGUI::drawAdvancedUI()
{
	if (ImGui::CollapsingHeader("Advanced")) {
		if (ImGui::Button("Clear Path"))
		{
			clearTraj();
		}

		if (pPanel.mControlStateRecord)
		{
			if (ImGui::Button("End Path Record"))
			{
				pPanel.sClient->sendRobotControlStateEndRecord();
			}
		}
		else {
			if (ImGui::Button("Begin Path Record"))
			{
				pPanel.sClient->sendRobotControlStateStartRecord();
			}
		}
	}

	if (ImGui::CollapsingHeader("SpotCam"))
	{
		if (pPanel.mSpotCamRecord == 0)
		{
			if (ImGui::Button("CamRecordStart"))
			{
				pPanel.sClient->sendRobotMsgStr("spotCamRecordStart");
			}
		}
		else if (pPanel.mSpotCamRecord == 1)
		{
			if (ImGui::Button("CamRecordStop"))
			{
				pPanel.sClient->sendRobotMsgStr("spotCamRecordStop");
			}
		}
		
		ImGui::SameLine();

		if (ImGui::Button("CamSnap"))
		{
			pPanel.sClient->sendRobotMsgStr("spotCamSnap");
		}

		ImGui::SameLine();

		if (ImGui::Button("CamRestart"))
		{
			pPanel.sClient->sendRobotMsgStr("spotCamRestart");
		}

		if (ImGui::Button("LightON"))
		{
			pPanel.sClient->sendRobotMsgStr("spotLightON");
		}
		ImGui::SameLine();
		if (ImGui::Button("LightOFF"))
		{
			pPanel.sClient->sendRobotMsgStr("spotLightOFF");
		}
	}
}

void bvlosGUI::drawAutonomyUI()
{
	if (ImGui::CollapsingHeader("Autonomy")) {
		std::vector<const char*> navFilenameStrs(pPanel.mSpotGraphNavFiles.size());
		for (size_t i = 0; i < pPanel.mSpotGraphNavFiles.size(); i++)
		{
			navFilenameStrs[i] = pPanel.mSpotGraphNavFiles[i].data();
		}

		ImGui::Combo("Graphs", &mSelectNavGraphIdx, navFilenameStrs.data(), (int)navFilenameStrs.size());
		nlohmann::json sJSON;
		if (ImGui::Button("List"))
		{			
			sJSON["cmd"] = "list";
			pPanel.sClient->sendRobotSpotGraphNav(sJSON);
		}

		if (pPanel.mSpotMode == bvlosPanel::SPOT_RUN_NORMAL_MODE)
		{
			ImGui::SameLine();
			if (ImGui::Button("Play"))
			{
				if ((mSelectNavGraphIdx >= 0) && (pPanel.mSpotGraphNavFiles.size() > 0))
				{
					sJSON["cmd"] = "playNav";
					sJSON["navFile"] = pPanel.mSpotGraphNavFiles[mSelectNavGraphIdx];
					std::string cMsg = "Trying to play nav file: " + pPanel.mSpotGraphNavFiles[mSelectNavGraphIdx];
					addLogText(cMsg);
					pPanel.sClient->sendRobotSpotGraphNav(sJSON);
					addLogText("Please wait while setting up Autonomy path...");
				}
			}
			ImGui::SameLine();
			if (ImGui::Button("Record"))
			{
				sJSON["cmd"] = "recordStart";
				pPanel.sClient->sendRobotSpotGraphNav(sJSON);
			}
		}
		else if (pPanel.mSpotMode == bvlosPanel::SPOT_RUN_NAV_RECORD_MODE)
		{
			ImGui::SameLine();
			if (ImGui::Button("End Record"))
			{
				sJSON["cmd"] = "recordStop";
				pPanel.sClient->sendRobotSpotGraphNav(sJSON);
			}
		}
		else if (pPanel.mSpotMode == bvlosPanel::SPOT_RUN_NAV_PLAY_MODE)
		{
			ImGui::SameLine();
			if (ImGui::Button("Stop Nav"))
			{
				sJSON["cmd"] = "stopNav";
				pPanel.sClient->sendRobotSpotGraphNav(sJSON);
			}
		}
	}
}

void bvlosGUI::drawPluginsUI()
{
	if (pPanel.mPluginManager) {
		pPanel.mPluginManager->drawGUI();
	}
}

void bvlosGUI::drawLogUI()
{
	if (mLogUI)
	{
		mLogUI->draw();
	}
}

void bvlosGUI::drawJoystick()
{
	std::array<float, 3> moveVals = { pPanel.mJoystickPos.x(), pPanel.mJoystickPos.y(), 0.0f };
	if (pPanel.mMovementMode == bvlosPanel::MOVEMENT_MODE::STAND)
	{
		moveVals = { pPanel.mJoystickOrientation.x(), pPanel.mJoystickOrientation.y(), pPanel.mJoystickOrientation.z() };
	}

	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	ImVec2 canvas_pos = ImGui::GetCursorScreenPos();            // ImDrawList API uses screen coordinates!
	ImVec2 canvas_size(150, 150);
	float rotPtX = canvas_pos.x + (canvas_size.x * 0.5f);
	float rotPtY = canvas_pos.y + (canvas_size.y * 0.5f);

	draw_list->AddRectFilledMultiColor(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), IM_COL32(50, 50, 50, 255), IM_COL32(50, 50, 60, 255), IM_COL32(60, 60, 70, 255), IM_COL32(50, 50, 60, 255));
	draw_list->AddRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), IM_COL32(255, 255, 255, 255));

	// Clip in window extents
	draw_list->PushClipRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), true);
	{
		// Axis lines
		draw_list->AddLine(
			ImVec2(canvas_pos.x, canvas_pos.y + (canvas_size.y * 0.5f)),
			ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + (canvas_size.y * 0.5f)),
			IM_COL32(255, 255, 0, 255));
		draw_list->AddLine(
			ImVec2(canvas_pos.x + (canvas_size.x * 0.5f), canvas_pos.y),
			ImVec2(canvas_pos.x + (canvas_size.x * 0.5f), canvas_pos.y + canvas_size.y),
			IM_COL32(255, 255, 0, 255));
	}

	{
		// Joystick pos
		float circX = canvas_size.x * ((moveVals[0] + 1.0f) * 0.5f);
		float circY = canvas_size.y * ((moveVals[1] + 1.0f) * 0.5f);
		draw_list->AddCircleFilled(ImVec2(canvas_pos.x + circX, canvas_pos.y + circY), 5.0f, IM_COL32(255, 100, 0, 255));
	}

	{
		// Keyboard controls
		auto charActiveColor = [this]() -> ImU32
		{
			return IM_COL32(50, 255, 255, 255);
		};

		auto isCharActive = [this]() -> bool
		{
			return false;
		};

		auto rotatePt2D = [this](float cx, float cy, float angle, float orgX, float orgY) -> Magnum::Vector2
		{
			float s = std::sin(angle);
			float c = std::cos(angle);

			// translate point back to origin:
			float relX = cx - orgX;
			float relY = cy - orgY;

			// rotate point
			float xnew = relX * c - relY * s;
			float ynew = relX * s + relY * c;

			// translate point back:
			xnew += orgX;
			ynew += orgY;
			return { xnew, ynew };
		};

		static float speedX_T = 0.0f, speedY_T = 0.0f;

		draw_list->AddText(ImVec2(canvas_pos.x + (canvas_size.x * 0.5f), canvas_pos.y), charActiveColor(), "W");
		draw_list->AddText(ImVec2(canvas_pos.x + (canvas_size.x * 0.5f), canvas_pos.y + canvas_size.y - 15.0f), charActiveColor(), "S");
		draw_list->AddText(ImVec2(canvas_pos.x, canvas_pos.y + (canvas_size.y * 0.5f)), charActiveColor(), "A");
		draw_list->AddText(ImVec2(canvas_pos.x + canvas_size.x - 10.0f, canvas_pos.y + (canvas_size.y * 0.5f)), charActiveColor(), "D");

		constexpr float turnRadius = 15.0f;
		if (pPanel.mMovementMode == bvlosPanel::MOVEMENT_MODE::STAND) {
			mVizRotAngle = pPanel.mYaw;
		}

		draw_list->AddCircle(ImVec2(rotPtX, rotPtY), turnRadius, IM_COL32(35, 180, 0, 255));
		for (int i = 0; i < 4; i++)
		{
			Magnum::Vector2 circPt(0, 0);
			switch (i)
			{
			case 0:
				circPt = rotatePt2D(rotPtX + turnRadius, rotPtY, mVizRotAngle, rotPtX, rotPtY);
				break;
			case 1:
				circPt = rotatePt2D(rotPtX - turnRadius, rotPtY, mVizRotAngle, rotPtX, rotPtY);
				break;
			case 2:
				circPt = rotatePt2D(rotPtX, rotPtY + turnRadius, mVizRotAngle, rotPtX, rotPtY);
				break;
			case 3:
				circPt = rotatePt2D(rotPtX, rotPtY - turnRadius, mVizRotAngle, rotPtX, rotPtY);
				break;
			default:
				break;
			}
			draw_list->AddCircle(ImVec2(circPt.x(), circPt.y()), 2.5f, IM_COL32(255, 235, 0, 255));
		}

		draw_list->AddText(ImVec2(canvas_pos.x + (canvas_size.x * 0.5f) - turnRadius * 1.45f, canvas_pos.y + (canvas_size.y * 0.5f) - 15.0f), charActiveColor(), "Q");
		draw_list->AddText(ImVec2(canvas_pos.x + (canvas_size.x * 0.5f) + turnRadius * 1.0f, canvas_pos.y + (canvas_size.y * 0.5f) - 15.0f), charActiveColor(), "E");
	}

	draw_list->PopClipRect();
}

void bvlosGUI::drawTrajectory()
{
	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	ImVec2 canvas_size(150, 150);
	ImVec2 canvas_pos = ImGui::GetCursorScreenPos();            // ImDrawList API uses screen coordinates!
	canvas_pos.x += canvas_size.x + 20;

	draw_list->AddRectFilledMultiColor(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), IM_COL32(50, 50, 50, 255), IM_COL32(50, 50, 60, 255), IM_COL32(60, 60, 70, 255), IM_COL32(50, 50, 60, 255));
	draw_list->AddRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), IM_COL32(255, 255, 255, 255));

	if (pPanel.mRobotFrameSet) {
		mTrajMin.x() = std::min(mTrajMin.x(), pPanel.mRobotPos.x());
		mTrajMin.y() = std::min(mTrajMin.y(), pPanel.mRobotPos.y());
		mTrajMin.z() = std::min(mTrajMin.z(), pPanel.mRobotPos.z());

		mTrajMax.x() = std::max(mTrajMax.x(), pPanel.mRobotPos.x());
		mTrajMax.y() = std::max(mTrajMax.y(), pPanel.mRobotPos.y());
		mTrajMax.z() = std::max(mTrajMax.z(), pPanel.mRobotPos.z());

		if (mTrajList.empty())
		{
			mTrajList.push_back(pPanel.mRobotPos);
		}
		else {
			auto curPos = mTrajList.back();
			auto diffPos = curPos - pPanel.mRobotPos;
			if (diffPos.length() > 0.1f)
			{
				mTrajList.push_back(pPanel.mRobotPos);
			}
		}
	}

	ImVec2 canvas_midpt;
	canvas_midpt.x = canvas_pos.x + (canvas_size.x * 0.5f);
	canvas_midpt.y = canvas_pos.y + (canvas_size.y * 0.5f);
	auto getPlotPt = [this, &canvas_pos, &canvas_midpt, &canvas_size](float x, float y) -> ImVec2
	{
		ImVec2 retPt(0, 0);
		float tWidth = mTrajMax.x() - mTrajMin.x();
		float tHeight = mTrajMax.y() - mTrajMin.y();
		float tMidX = mTrajMin.x() + (tWidth * 0.5f);
		float tMidY = mTrajMin.y() + (tHeight * 0.5f);

		retPt.x = (x - tMidX) / tWidth * (canvas_size.x * 0.5f) + canvas_midpt.x;
		retPt.y = (y - tMidY) / tHeight * (canvas_size.y * 0.5f) + canvas_midpt.y;
		return retPt;
	};

	for (size_t i = 1; i < mTrajList.size(); i++)
	{
		ImVec2 p1 = getPlotPt(mTrajList[i - 1].x(), mTrajList[i - 1].y());
		ImVec2 p2 = getPlotPt(mTrajList[i].x(), mTrajList[i].y());
		draw_list->AddLine(p1, p2, IM_COL32(35, 180, 0, 255), 1.0f);
	}

	if (!mTrajList.empty()) {
		auto lastPt = mTrajList.back();
		auto lastPlotPt = getPlotPt(lastPt.x(), lastPt.y());
		draw_list->AddCircleFilled(lastPlotPt, 5.0f, IM_COL32(255, 0, 0, 255), 12);

		auto rDir = pPanel.mRobotQuat.transformVector(Magnum::Vector3(15.0f, 0.0f, 0));
		auto lastPlotDirPt = lastPlotPt;
		lastPlotDirPt.x += rDir.x();
		lastPlotDirPt.y += rDir.y();
		draw_list->AddLine(lastPlotPt, lastPlotDirPt, IM_COL32(255, 100, 0, 255), 1.0f);
		draw_list->AddCircleFilled(lastPlotDirPt, 2.5f, IM_COL32(255, 255, 0, 255), 12);
	}
}

float bvlosGUI::totalTrajectoryDistance() const
{
	float retDist = 0;
	for (size_t i = 1; i < mTrajList.size(); i++)
	{
		retDist += (mTrajList[i] - mTrajList[i - 1]).length();
	}
	return retDist;
}

void bvlosGUI::clearTraj()
{
	mTrajList.clear();
}

int bvlosGUI::getActiveCamQuality() const
{
	auto& sceneManager = SceneManager::instance();
	int cCamQuality = pPanel.mCameraQuality;
	if ((sceneManager.activeSceneName() != "bvlosScene") && (sceneManager.activeSceneName() != "spotCamScene"))
	{
		auto cPlugin = pPanel.mPluginManager->getClientWithLayerStr(sceneManager.activeSceneName());
		if (cPlugin)
		{
			cCamQuality = cPlugin->getStreamCameraQuality();
		}
	}
	return cCamQuality;

}

void bvlosGUI::setActiveCamQuality(int valIn)
{
	auto& sceneManager = SceneManager::instance();
	if ((sceneManager.activeSceneName() == "bvlosScene") || (sceneManager.activeSceneName() == "spotCamScene"))
	{
		// Default camera quality
		pPanel.mCameraQuality = valIn;
		pPanel.updateRobotSettings();
	}
	else {
		auto cPlugin = pPanel.mPluginManager->getClientWithLayerStr(sceneManager.activeSceneName());
		if (cPlugin)
		{
			cPlugin->setStreamCameraQuality(valIn);
		}
	}
}

} // end of namespace DosPilotApp