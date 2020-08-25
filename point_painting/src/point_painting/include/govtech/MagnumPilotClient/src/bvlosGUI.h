#include <Magnum/Math/Vector2.h>
#include <Magnum/Math/Vector3.h>
#include <Magnum/Math/Quaternion.h>
#include <Magnum/Magnum.h>
#include <memory>
#include <vector>
#include <string>

namespace DosAutonomy
{

class bvlosPanel;
class logUI;
class bvlosGUI
{
public:
	enum class UI_MODE
	{
		NORMAL,
		SIMPLE
	};

public:
	bvlosGUI(bvlosPanel& pPanelIn);
	virtual ~bvlosGUI();

	void update();

	void setUIMode(UI_MODE modeIn) { mUIMode = modeIn; }
	UI_MODE getUIMode() const { return mUIMode; }

	void addLogText(const std::string& msgIn);

protected:
	void runUI();
	void drawMainUI();
	void drawCameraUI();
	void drawPowerUI();
	void drawDetectionUI();
	void drawMovementUI();
	void drawMotionParamsUI();
	void drawMotionParamsSimpleUI();
	void drawAdvancedUI();
	void drawAutonomyUI();
	void drawPluginsUI();
	void drawLogUI();

	void drawJoystick();
	void drawTrajectory();

	float totalTrajectoryDistance() const;
	void clearTraj();

	int getActiveCamQuality() const;
	void setActiveCamQuality(int valIn);

protected:
	bvlosPanel& pPanel;
	std::unique_ptr<logUI> mLogUI;
	float mVizRotAngle = 0;
	bool mRecordVideo = false;
	Magnum::Vector3 mTrajMin = { 0, 0, 0 };
	Magnum::Vector3 mTrajMax = { 0, 0, 0 };
	std::vector<Magnum::Vector3> mTrajList;
	UI_MODE mUIMode = UI_MODE::NORMAL;
	int mSelectNavGraphIdx = 0;
};

} // end of namespace DosPilotApp