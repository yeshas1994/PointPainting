#pragma once
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/TranslationRotationScalingTransformation2D.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>

namespace DosAutonomy
{

typedef Magnum::SceneGraph::Object<Magnum::SceneGraph::TranslationRotationScalingTransformation2D> Object2D;
typedef Magnum::SceneGraph::Scene<Magnum::SceneGraph::TranslationRotationScalingTransformation2D> Scene2D;

typedef Magnum::SceneGraph::Object<Magnum::SceneGraph::MatrixTransformation3D> Object3D;
typedef Magnum::SceneGraph::Scene<Magnum::SceneGraph::MatrixTransformation3D> Scene3D;

}