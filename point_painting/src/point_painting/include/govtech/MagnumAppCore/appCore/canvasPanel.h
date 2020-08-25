#pragma once
#include <unordered_map>
#include <string>
#include <memory>
#include <functional>
#include <chrono>
#include <Magnum/Math/Vector2.h>
#include <Magnum/Magnum.h>

namespace DosAutonomy
{

// A class to perform UI/Gameplay operations, representing a "panel" or "canvas" screen
class CanvasPanel
{
public:
	CanvasPanel();
	virtual ~CanvasPanel();

	void runInit();
	bool isInit() const { return _isInit; }

	// Override for any initialization purposes
	virtual void init() {}
	// Override for the canvas ick event
	virtual void canvasTick(float) {}
	// Override for drawing imgui tick event
	virtual void imguiTick(float) {}
	// Override for 3D rendering tick event
	virtual void scene3DTick(float) {}
	// Input events
	virtual void evtKeyDown(const std::string&) {}
	virtual void evtKeyUp  (const std::string&) {}
	virtual void onMouseDown(int, const Magnum::Vector2i&) {}
	virtual void onMouseUp  (int, const Magnum::Vector2i&) {}
	virtual void onMouseMove(const Magnum::Vector2i&, const Magnum::Vector2i&) {}
	virtual void onMouseScroll(float) {}

protected:
	bool _isInit = false;
};

// The manager class tha manages all CanvasPanel objects
class CanvasPanelManager
{
public:
	CanvasPanelManager() = default;

	// Adds a new canvas panel
	bool addCanvasPanel(const std::string& nameIn, std::unique_ptr<CanvasPanel>& panelIn);
	// Remove a canvas panel
	void removeCanvasPanel(const std::string& nameIn);
	// Clear all canvas
	void clear();

	// Sets/Returns the current Window Size
	void setWinSize(const Magnum::Vector2i& wSizeIn);
	const Magnum::Vector2i& getWinSize() const;
	// Sets/Returns the current Frame Buffer Size
	void setFrameBufferSize(const Magnum::Vector2i& wSizeIn);
	const Magnum::Vector2i& getFrameBufferSize() const;

	// Returns a canvas panel by name
	CanvasPanel* getCanvasPanel(const std::string& nameIn) const;
	// Returns the active canvas panel
	CanvasPanel* getActiveCanvasPanel() const;
	// Sets the active canvas
	void setActiveCanvasPanel(const std::string& nameIn);	
	// Ticks the active canvas
	void tickActiveCanvasPanel(float delta_time);

	// Returns the singleton instance of CanvasPanelManager
	static CanvasPanelManager& instance() {
		static CanvasPanelManager cInstance;
		return cInstance;
	}

	// Returns current time in MS
	static std::chrono::milliseconds getTimeMS() {
		return std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch()
		);
	}

	// Returns if a the file exists
	static bool fileExists(const std::string& filename);
	// Returns the entire contents of a file as a string
	static std::string fileReadAsString(const std::string& filename);

	// Setup callback functions to setup imgui rendering
	void setImGuiFns(std::function<void()> startFn, std::function<void()> endFn);

protected:
	std::unordered_map<std::string, std::unique_ptr<CanvasPanel>> _panelMap;
	std::string _activeCanvasName;
	CanvasPanel* _activeCanvas = nullptr;
	std::function<void()> _imguiStartFn, _imguiEndFn;
	Magnum::Vector2i _winSize = Magnum::Vector2i(0, 0);
	Magnum::Vector2i _frameBufferSize = Magnum::Vector2i(0, 0);
};

} // end of namespace DosAutonomy