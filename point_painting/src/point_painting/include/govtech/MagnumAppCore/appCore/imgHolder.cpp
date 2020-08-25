#include <Corrade/Containers/StridedArrayView.h>
#include <appCore/imgHolder.h>

namespace DosAutonomy
{
ImgHolder:: ~ImgHolder()
{
}
ImgHolder::ImgHolder(
	const std::string& nameIn, 
	int texWidth,
	int texHeight,
	Object2D& object,
	Magnum::SceneGraph::DrawableGroup2D& drawables)
	: DrawHolder(nameIn, object, drawables)

{
	_texture = new Magnum::GL::Texture2D;
	// Texture setup
	Magnum::Containers::Array<char> imgData(texWidth * texHeight * sizeof(uint8_t) * 4);
	for (size_t i = 0; i < imgData.size(); i+=4)
	{
		imgData[i] = 0;
		imgData[i + 1] = 0;
		imgData[i + 2] = 0;
		imgData[i + 3] = 127;
	}
	//_img = std::shared_ptr<Magnum::Image2D>(new Magnum::Image2D(Magnum::PixelFormat::RGBA8Unorm, { texWidth, texHeight }, std::move(imgData)));
	_img = new Magnum::Image2D( Magnum::PixelFormat::RGBA8Unorm, { texWidth, texHeight }, std::move( imgData ) );
	_texture->setWrapping(Magnum::GL::SamplerWrapping::ClampToEdge)
		.setMagnificationFilter(Magnum::GL::SamplerFilter::Linear)
		.setMinificationFilter(Magnum::GL::SamplerFilter::Linear)
		.setStorage(1, Magnum::GL::textureFormat(_img->format()), _img->size())
		.setSubImage(0, {}, *_img);

	// Setup quad
	struct MeshVertex {
		Magnum::Vector2 position;
		Magnum::Vector2 textureCoordinates;
	};

	const MeshVertex data[]{
		{{-0.5f, -0.5f}, {0.0f, 0.0f}},
		{{ -0.5f, 0.5f}, {0.0f, 1.0f}},
		{ { 0.5f, -0.5f}, {1.0f, 0.0f}},
		{{ 0.5f,  0.5f}, {1.0f, 1.0f}}
	};

	Magnum::GL::Buffer buffer;
	buffer.setData(data);
	
	_mesh->setCount(4)
		.setPrimitive(Magnum::GL::MeshPrimitive::TriangleStrip)
		.addVertexBuffer(std::move(buffer), 0,
			Magnum::Shaders::Flat2D::Position{},
			Magnum::Shaders::Flat2D::TextureCoordinates{});
}

Magnum::Image2D& ImgHolder::getImg()
{
	return *_img;
}

void ImgHolder::foreachImgPixel(std::function<void(Magnum::Color4ub&, int, int, int, int)>& fnIn)
{
	Magnum::Containers::StridedArrayView2D<Magnum::Color4ub> pixels = _img->pixels<Magnum::Color4ub>();
	size_t iWidth = _img->size().x();
	size_t iHeight = _img->size().y();
	int cx = 0, cy = 0;
	for (auto row : pixels.slice({ 0, 0 }, { iHeight, iWidth })) {
		for (Magnum::Color4ub& pixel : row) {
			fnIn(pixel, cx, cy, iWidth, iHeight);
			cx++;
		}
		cx = 0;
		cy++;
	}
}

void ImgHolder::setWithCVMatPixels(const cv::Mat& imMat)
{
	int imgWidth = imMat.cols, imgHeight = imMat.rows, imgBpp = imMat.channels();
	int iWidth = int(_img->size().x());
	int iHeight = int(_img->size().y());
	if ((imgWidth == iWidth) && (imgHeight == iHeight))
	{
		int cx = 0, cy = 0;
		uint8_t * rawBytes = imMat.data;
		Magnum::Containers::StridedArrayView2D<Magnum::Color4ub> pixels = _img->pixels<Magnum::Color4ub>();
		for (auto row : pixels.slice({ 0, 0 }, { size_t(iHeight), size_t(iWidth) })) {
			for (Magnum::Color4ub& pixel : row) {
				uint8_t* basePtr = rawBytes + (sizeof(uint8_t) * imgBpp * imgWidth * (iHeight - 1 - cy)) + (sizeof(uint8_t) * imgBpp * cx);
				uint8_t r = 0, g = 0, b = 0;
				if (imgBpp == 3) {
					// color
					r = basePtr[0];
					g = basePtr[1];
					b = basePtr[2];
				}
				else if (imgBpp == 1) {
					// black and white
					r = basePtr[0];
					g = r;
					b = r;
				}
				pixel = { r, g, b, 255 };
				cx++;
			}
			cx = 0;
			cy++;
		}
	}
}

void ImgHolder::clearPixels(const Magnum::Color4ub& clearColor)
{
	Magnum::Containers::StridedArrayView2D<Magnum::Color4ub> pixels = _img->pixels<Magnum::Color4ub>();
	size_t iWidth = _img->size().x();
	size_t iHeight = _img->size().y();
	for (auto row : pixels.slice({ 0, 0 }, { iHeight, iWidth })) {
		for (Magnum::Color4ub& pixel : row) {
			pixel = clearColor;
		}
	}
}

void ImgHolder::flagDirty()
{
	std::lock_guard<std::mutex> scopeLock(_texLock);
	_isDirty = true;
}

void ImgHolder::updateTexture()
{
	if (_isDirty) {
		std::lock_guard<std::mutex> scopeLock(_texLock);
		_texture->setSubImage(0, {}, *_img);
		_isDirty = false;
	}
}

std::shared_ptr<ImgHolder> ImgHolder::createImageObject(
	const std::string& nameIn,
	int texWidth,
	int texHeight,
	const Magnum::Vector2& posIn,
	const Magnum::Vector2& scaleIn, 
	SceneManager::SceneNode& sNode)
{
	auto newObj = new Object2D{ &sNode._scene2D };
	newObj->setTranslation(posIn);
	newObj->setScaling(scaleIn);
	return std::make_shared<ImgHolder>(nameIn, texWidth, texHeight, *newObj, sNode._drawables2D);
}

void ImgHolder::draw(const Magnum::Matrix3& transformationMatrix, Magnum::SceneGraph::Camera2D& camera)
{
	if ( _shader == nullptr || _mesh == nullptr )
		return;
	// Override with your own custom drawing functions
	std::lock_guard<std::mutex> scopeLock(_texLock);
	{
		_shader->bindTexture(*_texture).setTransformationProjectionMatrix(camera.projectionMatrix() * transformationMatrix);
		_mesh->draw( *_shader );
	}
}

} // end of namespace DosAutonomy