#pragma once
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Math/Color.h>

#include <opencv2/opencv.hpp>
#include <appCore/sceneManager.h>
#include <mutex>

namespace DosAutonomy
{

class ImgHolder : public DrawHolder
{
public:
	explicit ImgHolder(
		const std::string& nameIn,
		int texWidth,
		int texHeight,
		Object2D& object,
		Magnum::SceneGraph::DrawableGroup2D& drawables);

	virtual ~ImgHolder();
	
	Magnum::Image2D& getImg();
	// Allows you to easily iterate through each and every pixel of the texture, will automatically update the texture after pixel modification
	// Callback function parameters: (Pixel Color, X, Y, Width, Height)
	void foreachImgPixel(std::function<void(Magnum::Color4ub&, int, int, int, int)>& fnIn);
	// Sets the texture with an OpenCV Image Mat
	void setWithCVMatPixels(const cv::Mat& imMat);
	// Clears the texture with a color
	void clearPixels(const Magnum::Color4ub& clearColor);
	// Flag it dirty for a texture update
	void flagDirty();
	// Update texture to GPU memory
	void updateTexture();

	// Returns an image object ready for rendering and added into the scene graph
	static std::shared_ptr<ImgHolder> createImageObject(
		const std::string& nameIn,
		int texWidth,
		int texHeight,
		const Magnum::Vector2& posIn, 
		const Magnum::Vector2& scaleIn, 
		SceneManager::SceneNode& sNode);

protected:
	void draw(const Magnum::Matrix3& transformationMatrix, Magnum::SceneGraph::Camera2D& camera) override;

protected:
	Magnum::GL::Texture2D* _texture;
	Magnum::Image2D* _img = nullptr;
	std::mutex _texLock;
	bool _isDirty = false;
};

} // end of namespace DosAutonomy