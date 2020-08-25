#include "Camera.h"
#include <Magnum/GL/DefaultFramebuffer.h>

namespace DosAutonomy
{

// Camera objects
Camera2D::Camera2D(Scene2D* scene) :
    Object2D(scene),
    Magnum::SceneGraph::Camera2D(*this)
{
    setAspectRatioPolicy(Magnum::SceneGraph::AspectRatioPolicy::Extend)
        .setProjectionMatrix(Magnum::Matrix3::projection({ 20.0f, 20.0f }))
        .setViewport(Magnum::GL::defaultFramebuffer.viewport().size());
}

Camera3D::Camera3D(Scene3D* scene) :
    Object3D(scene),
    Magnum::SceneGraph::Camera3D(*this)
{
    using Magnum::Math::Literals::operator""_degf;
    setAspectRatioPolicy(Magnum::SceneGraph::AspectRatioPolicy::Extend)
        .setProjectionMatrix(Magnum::Matrix4::perspectiveProjection(45.0_degf, Magnum::Vector2(1600.f, 900.f).aspectRatio(), 0.01f, 1000.0f))
        .setViewport(Magnum::GL::defaultFramebuffer.viewport().size());
}

void Camera3D::update()
{
    if (_attached) {
        Magnum::Matrix4 objectTransform(_attached->transformation());
        objectTransform[0].xyz() = objectTransform[0].xyz().normalized();
        objectTransform[1].xyz() = objectTransform[1].xyz().normalized();
        objectTransform[2].xyz() = objectTransform[2].xyz().normalized();
        setTransformation(objectTransform * _relativeTransform);
    }
}

void Camera3D::attach(Object3D* obj)
{
    _attached = obj;
}

void Camera3D::zoom(float distance)
{
    if (_attached) {  // If attached, nove camera closer to/further from object
        if (!_arm.isZero())
            _relativeTransform[3].xyz() -= distance * _arm;
    }
    else // else move along the cameras direction
        translate(distance *-transformation()[2].xyz());
}

void Camera3D::setRelativeTransform(const Magnum::Matrix4& relativeTransformation)
{
    _relativeTransform = relativeTransformation;
    _arm = _relativeTransform[3].xyz();
    if (!_arm.isZero())
        _arm = _arm.normalized();
}

}