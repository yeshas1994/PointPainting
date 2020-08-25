#include <appCore/canvasPanel.h>
#include <Magnum/GL/Renderer.h>
#include <iostream>
#include <fstream>
#include <string>

namespace DosAutonomy
{

CanvasPanel::CanvasPanel()
{
}

CanvasPanel::~CanvasPanel()
{
}

void CanvasPanel::runInit()
{
	if (!_isInit)
	{
		_isInit = true;
		init();
	}
}

bool CanvasPanelManager::addCanvasPanel(const std::string& nameIn, std::unique_ptr<CanvasPanel>& panelIn)
{
	if (_panelMap.count(nameIn) > 0)
	{
		return false;
	}

	_panelMap[nameIn] = std::move(panelIn);
	return true;
}

void CanvasPanelManager::removeCanvasPanel(const std::string& nameIn)
{
	auto findIter = _panelMap.find(nameIn);
	if (findIter->second.get() == _activeCanvas) {
		_activeCanvas = nullptr;
		_activeCanvasName.clear();
	}
	_panelMap.erase(findIter);
}

void CanvasPanelManager::clear()
{
	_panelMap.clear();
	_activeCanvas = nullptr;
}

void CanvasPanelManager::setWinSize(const Magnum::Vector2i& wSizeIn)
{
	_winSize = wSizeIn;
}

const Magnum::Vector2i& CanvasPanelManager::getWinSize() const
{
	return _winSize;
}

void CanvasPanelManager::setFrameBufferSize(const Magnum::Vector2i& wSizeIn)
{
	_frameBufferSize = wSizeIn;
}

const Magnum::Vector2i& CanvasPanelManager::getFrameBufferSize() const
{
	return _frameBufferSize;
}

CanvasPanel* CanvasPanelManager::getCanvasPanel(const std::string& nameIn) const
{
	auto findIter = _panelMap.find(nameIn);
	return (findIter == _panelMap.end()) ? nullptr : findIter->second.get();
}

CanvasPanel* CanvasPanelManager::getActiveCanvasPanel() const
{
	return _activeCanvas;
}

void CanvasPanelManager::setActiveCanvasPanel(const std::string& nameIn)
{
	_activeCanvas = nullptr;
	_activeCanvasName = "";
	auto cPanel = getCanvasPanel(nameIn);
	if (cPanel)
	{
		_activeCanvasName = nameIn;
		_activeCanvas = cPanel;
	}
}

void CanvasPanelManager::tickActiveCanvasPanel(float delta_time)
{
	if (_activeCanvas == nullptr)
	{
		return;
	}

	if (!_activeCanvas->isInit())
	{
		_activeCanvas->runInit();
	}

	// 3D Rendering
	Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::Blending);
	Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::ScissorTest);
	Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::FaceCulling);
	Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::DepthTest);
	_activeCanvas->scene3DTick(delta_time);

	// 2D Canvas
	Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::Blending);
	Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::ScissorTest);
	Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::FaceCulling);
	Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::DepthTest);
	_activeCanvas->canvasTick(delta_time);

	// ImGUI
	if (_imguiStartFn && _imguiEndFn) {
		_imguiStartFn();

		Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::Blending);
		Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::ScissorTest);
		Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::FaceCulling);
		Magnum::GL::Renderer::disable(Magnum::GL::Renderer::Feature::DepthTest);
		_activeCanvas->imguiTick(delta_time);

		_imguiEndFn();
	}
}

bool CanvasPanelManager::fileExists(const std::string& filename)
{
	std::ifstream f(filename.c_str());
	return f.good();
}

std::string CanvasPanelManager::fileReadAsString(const std::string& filename)
{
	const std::string cFilename(filename);
	if (!fileExists(cFilename))
	{
		return "";
	}

	std::ifstream cStream(cFilename.c_str());
	std::string fileStr((std::istreambuf_iterator<char>(cStream)),
		std::istreambuf_iterator<char>());
	return fileStr;
}

void CanvasPanelManager::setImGuiFns(std::function<void()> startFn, std::function<void()> endFn)
{
	_imguiStartFn = startFn;
	_imguiEndFn = endFn;
}

} // end of namespace DosAutonomy