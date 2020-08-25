#pragma once
#include <functional>
#include <mutex>
#include "Camera.h"
#include <Magnum/Shaders/Flat.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/Math/DualComplex.h>
#include <memory>
#include <string>
#include <unordered_map>

namespace DosAutonomy
{

// DrawHolder
class DrawHolder : public Magnum::SceneGraph::Drawable2D {
public:
	explicit DrawHolder(
		const std::string& nameIn,
		Object2D& object,
		Magnum::SceneGraph::DrawableGroup2D& drawables )
		: _name( nameIn )
		, Magnum::SceneGraph::Drawable2D{ object, &drawables }

	{
		_shader = new Magnum::Shaders::Flat2D( Magnum::Shaders::Flat2D::Flag::Textured );
		_mesh = new Magnum::GL::Mesh;
	}

	virtual ~DrawHolder() 
	{	}

	const std::string& name() const { return _name; }
	void setName(const std::string& nameIn) { _name = nameIn; }

protected:
	std::string _name;
	Magnum::GL::Mesh* _mesh;
	Magnum::Shaders::Flat2D* _shader;
};

// SceneManager
class SceneManager
{
public:
	class SceneNode
	{
	public:
		SceneNode();

		template<class T>
		void forEachChildNode(std::function<void(T&)>& fnIn)
		{
			Object2D& sceneObj = *(dynamic_cast<Object2D*>(&_scene2D));
			forEachChildNodeWorker(sceneObj, fnIn);
		}

	protected:
		template<class T>
		void forEachChildNodeWorker(Object2D& parentObj, std::function<void(T&)>& fnIn)
		{
			for (Object2D& cObj : parentObj.children())
			{
				T * tObj = dynamic_cast<T*>(&cObj);
				if (tObj)
				{
					fnIn(*tObj);
				}
				forEachChildNodeWorker<T>(cObj, fnIn);
			}
		}

	public:
		Scene2D _scene2D;
		Camera2D _camera2D;
		Magnum::SceneGraph::DrawableGroup2D _drawables2D;

		Scene3D _scene3D;
		Camera3D _camera3D;
		Magnum::SceneGraph::DrawableGroup3D _drawables3D;
	};

public:
	SceneManager() = default;
	void ExecuteBeforeDraw( const std::function<void()>& op ) { _op = op; }
	SceneNode* addScene(const std::string& nameIn);
	SceneNode* getScene(const std::string& nameIn) const;
	SceneNode* getActiveScene() const;
	bool removeScene(const std::string& nameIn);

	const std::string& activeSceneName() const;
	bool setActiveSceneName(const std::string& nameIn);

	bool drawActiveScene2D();
	bool drawActiveScene3D();
	void drawScene2D(SceneNode* scene);
	void drawScene3D(SceneNode* scene);
	std::mutex& getMutex();
	

	// Returns the Singleton Instance of SceneManager
	static SceneManager& instance() {
		static SceneManager retInstance;
		return retInstance;
	}

	template <class T>
	static T * findDrawHolderWithName(SceneNode& sceneNodeIn, const std::string& nameIn)
	{
		for (size_t i = 0; i < sceneNodeIn._drawables2D.size(); i++)
		{
			Magnum::SceneGraph::Drawable2D& cDrawable = sceneNodeIn._drawables2D[i];
			auto cDHolder = dynamic_cast<T*>(&cDrawable);
			if (cDHolder)
			{
				if (cDHolder->name() == nameIn) {
					return cDHolder;
				}
			}
		}
		return nullptr;
	}

protected:
	std::function<void()> _op;
	std::unordered_map<std::string, std::shared_ptr<SceneNode>> _sceneMap;
	std::string _activeSceneName;
	SceneNode*  _activeSceneNode = nullptr;
	std::mutex _mutex;
};

} // end of namespace DosAutonomy