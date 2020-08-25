#pragma once
#include <glm/glm.hpp>
#include <chrono>
#include <unordered_map>
#include <string>

namespace DosClient
{

class FollowControl
{
public:
	FollowControl() = default;

	void setMoveK1(float valIn);
	float getMoveK1() const;

	void setMoveK2(float valIn);
	float getMoveK2() const;

	void setStopAreaFrac(float valIn);
	float getStopAreaFrac() const;

	void setForwardsFrac(float valIn);
	float getForwardsFrac() const;

	void setRotDXCutoff(float valIn);
	float getRotDXCutoff() const;

	void setKSwitchCutoff(float valIn);
	float getKSwitchCutoff() const;

	void setMaxMoveSpeed(float valIn);
	float getMaxMoveSpeed() const;

	void setRotBlend(float valIn);
	float getRotBlend() const;

	glm::vec3 computeFollowFreeMove(const glm::vec2& imgSize, const glm::vec2& trackPt1, const glm::vec2& trackPt2);

protected:
	float interpScalar(float val1, float val2, float fraction) const;

protected:
	float m_k1 = 0.1f;
	float m_k2 = 0.2f;
	float m_stopAreaFrac = 0.35f;
	float m_forwardsFrac = 1.0f;
	float m_rotDXCutoff = 50.0f;
	float m_kSwitchCutoff = 150.0f;
	float m_maxMoveSpeed = 0.5f;
	float m_rotBlend = 0.1f;
};

} // end of namespace DosClient