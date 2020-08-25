#pragma once
#include "Scene.h"
#include <Magnum/SceneGraph/Camera.h>

namespace DosAutonomy
{

// Camera2D
class Camera2D : public Object2D, public Magnum::SceneGraph::Camera2D
{
public:
	Camera2D(Scene2D* scene);
};


// Camera3D
class Camera3D : public Object3D, public Magnum::SceneGraph::Camera3D
{
public:
	Camera3D(Scene3D* scene);

	void update();

	void attach(Object3D* obj);
	void zoom(float distance);
	void setRelativeTransform(const Magnum::Matrix4& relativeTransformation);

protected:
	Object3D* _attached = nullptr;
	Magnum::Matrix4 _relativeTransform;
	Magnum::Vector3 _arm;
};

}