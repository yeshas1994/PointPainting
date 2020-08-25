#include <appCore/sceneManager.h>

namespace DosAutonomy
{

// SceneNode
SceneManager::SceneNode::SceneNode()
    : _camera2D(&_scene2D)
    , _camera3D(&_scene3D)
{
}

// SceneManager
SceneManager::SceneNode* SceneManager::addScene(const std::string& nameIn)
{
    if (_sceneMap.count(nameIn) > 0)
        return nullptr;

    // Add scene
    auto newNode = std::make_unique<SceneNode>();
    _sceneMap[nameIn] = std::move(newNode);
    return _sceneMap[nameIn].get();
}

SceneManager::SceneNode* SceneManager::getScene(const std::string& nameIn) const
{
    auto findIter = _sceneMap.find(nameIn);
    return (findIter == _sceneMap.end()) ? nullptr : findIter->second.get();
}

SceneManager::SceneNode* SceneManager::getActiveScene() const
{
	auto cScene = getScene(activeSceneName());
	return cScene;
}

bool SceneManager::removeScene(const std::string& nameIn)
{
    return _sceneMap.erase(nameIn) > 0;
}

bool SceneManager::setActiveSceneName(const std::string& nameIn)
{
    _activeSceneName = nameIn;
    _activeSceneNode = getScene(_activeSceneName);

    if (_activeSceneNode == nullptr)
    {
        _activeSceneName = "";
        return false;
    }

    return true;
}

const std::string& SceneManager::activeSceneName() const
{
    return _activeSceneName;
}

bool SceneManager::drawActiveScene2D()
{
    if (_activeSceneNode == nullptr)
        return false;
    if ( _op )
    {
        _op();
        _op = nullptr;
    }
    std::lock_guard<std::mutex> lock( _mutex );
    _activeSceneNode->_camera2D.draw(_activeSceneNode->_drawables2D);
    return true;
}

bool SceneManager::drawActiveScene3D()
{
    if (_activeSceneNode == nullptr)
        return false;

    _activeSceneNode->_camera3D.draw(_activeSceneNode->_drawables3D);
    return true;
}

void SceneManager::drawScene2D(SceneNode* scene) {
    scene->_camera2D.draw(scene->_drawables2D);
}

void SceneManager::drawScene3D(SceneNode* scene) {
    scene->_camera3D.update();
    scene->_camera3D.draw(scene->_drawables3D);
}

std::mutex& SceneManager::getMutex()
{
    return _mutex;
}

} // end of namespace DosAutonomy