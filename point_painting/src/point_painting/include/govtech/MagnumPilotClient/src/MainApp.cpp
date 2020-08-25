#include <Magnum/GL/DefaultFramebuffer.h>

#include <Magnum/Platform/Sdl2Application.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Containers/StridedArrayView.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Version.h>
#include <Magnum/Math/Color.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Shaders/Flat.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Timeline.h>
#include <imguiLib/Context.hpp>
#include <Corrade/Utility/ConfigurationValue.h>
#include <Corrade/Utility/Arguments.h>
#include <appCore/imgHolder.h>
#include <appCore/sceneManager.h>
#include <appCore/canvasPanel.h>
#include <memory>
#include <array>

#include "bvlosPanel.h"

using namespace Magnum;

class MainApp: public Platform::Application {
public:
    explicit MainApp(const Arguments& arguments);

private:
	void exitEvent(ExitEvent& event) override;

    void drawEvent() override;
	void viewportEvent(ViewportEvent& event) override;
	void keyPressEvent(KeyEvent& event) override;
	void keyReleaseEvent(KeyEvent& event) override;

	void mousePressEvent(MouseEvent& event) override;
	void mouseReleaseEvent(MouseEvent& event) override;
	void mouseMoveEvent(MouseMoveEvent& event) override;
	void mouseScrollEvent(MouseScrollEvent& event) override;
	void textInputEvent(TextInputEvent& event) override;

	void addCanvasPanels();
	void tickCanvasPanels();

public:
	static constexpr Vector2i m_winSize{ 1600, 850 };

private:
	ImGuiIntegration::Context _imgui{ NoCreate };
	Magnum::Timeline _time;
	bool _run = true;
};

MainApp::MainApp(const Arguments& arguments):
    Platform::Application{arguments}
{	
    using namespace Math::Literals;
	setMinWindowSize(m_winSize);
	GL::defaultFramebuffer.setViewport({ {}, m_winSize });
	
	setWindowTitle("DOSS Pilot");
	GL::Renderer::setClearColor({ 0.3f, 0.3f, 0.3f, 1.0f });

    Debug{} << "Hello! This application is running on"
        << GL::Context::current().version() << "using"
        << GL::Context::current().rendererString();

	addCanvasPanels();

	// GL stuff
	GL::Renderer::setBlendEquation(GL::Renderer::BlendEquation::Add,
		GL::Renderer::BlendEquation::Add);
	GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::SourceAlpha,
		GL::Renderer::BlendFunction::OneMinusSourceAlpha);

	// ImGui
	_imgui = ImGuiIntegration::Context(Vector2{ windowSize() } / dpiScaling(),
		windowSize(), framebufferSize());
	auto& canvasManager = DosAutonomy::CanvasPanelManager::instance();
	canvasManager.setImGuiFns(
		[this]() -> void
		{
			_imgui.newFrame();

			/* Enable text input, if needed */
			if (ImGui::GetIO().WantTextInput && !isTextInputActive())
				startTextInput();
			else if (!ImGui::GetIO().WantTextInput && isTextInputActive())
				stopTextInput();
		},
		[this]() -> void
		{
			_imgui.drawFrame();
		}
	);

    // Other setup
	_time.start();
	setSwapInterval(1);
    setMinimalLoopPeriod(1);
}

void MainApp::exitEvent(ExitEvent&) {
	DosAutonomy::CanvasPanelManager::instance().clear();
	_run = false;
	exit();
}

void MainApp::drawEvent() {
	if (_run)
	{
		GL::defaultFramebuffer.clear(GL::FramebufferClear::Color);
		// Render
		tickCanvasPanels();
		swapBuffers();
		redraw();

		_time.nextFrame();
	}
}

void MainApp::viewportEvent(ViewportEvent& event)
{
	GL::defaultFramebuffer.setViewport({ {}, event.framebufferSize() });
}

void MainApp::keyPressEvent(KeyEvent& event) {
	_imgui.handleKeyPressEvent(event);

	auto& canvasManager = DosAutonomy::CanvasPanelManager::instance();
	auto cPanel = canvasManager.getActiveCanvasPanel();
	if (cPanel)
	{
		auto cStr = event.keyName();
		if (cStr.length() > 0) {
			cPanel->evtKeyDown(cStr);
		}
	}
}

void MainApp::keyReleaseEvent(KeyEvent& event) {
	if (_imgui.handleKeyReleaseEvent(event)) return;	
}

void MainApp::mousePressEvent(MouseEvent& event) {
	if (_imgui.handleMousePressEvent(event)) return;
	auto& canvasManager = DosAutonomy::CanvasPanelManager::instance();
	auto cPanel = canvasManager.getActiveCanvasPanel();
	if (cPanel)
	{
		cPanel->onMouseDown(static_cast<int>(event.button()), event.position());
	}
}

void MainApp::mouseReleaseEvent(MouseEvent& event) {
	if (_imgui.handleMouseReleaseEvent(event)) return;
	auto& canvasManager = DosAutonomy::CanvasPanelManager::instance();
	auto cPanel = canvasManager.getActiveCanvasPanel();
	if (cPanel)
	{
		cPanel->onMouseUp(static_cast<int>(event.button()), event.position());
	}
}

void MainApp::mouseMoveEvent(MouseMoveEvent& event) {
	if (_imgui.handleMouseMoveEvent(event)) return;
	auto& canvasManager = DosAutonomy::CanvasPanelManager::instance();
	auto cPanel = canvasManager.getActiveCanvasPanel();
	if (cPanel)
	{
		cPanel->onMouseMove(event.position(), event.relativePosition());
	}
}

void MainApp::mouseScrollEvent(MouseScrollEvent& event) {
	if (_imgui.handleMouseScrollEvent(event)) {
		/* Prevent scrolling the page */
		event.setAccepted();
		return;
	}
}

void MainApp::textInputEvent(TextInputEvent& event) {
	if (_imgui.handleTextInputEvent(event)) return;
}

void MainApp::addCanvasPanels()
{
	auto& canvasManager = DosAutonomy::CanvasPanelManager::instance();
	canvasManager.setWinSize(m_winSize);
	canvasManager.setFrameBufferSize(framebufferSize());

	std::unique_ptr<DosAutonomy::CanvasPanel> bvlosPanel = std::make_unique<DosAutonomy::bvlosPanel>();
	canvasManager.addCanvasPanel("bvlos", bvlosPanel);

	canvasManager.setActiveCanvasPanel("bvlos");
}

void MainApp::tickCanvasPanels()
{
	auto& canvasManager = DosAutonomy::CanvasPanelManager::instance();
	canvasManager.tickActiveCanvasPanel(_time.previousFrameDuration());
}

MAGNUM_APPLICATION_MAIN(MainApp)