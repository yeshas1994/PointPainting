#include <dosAutonomy/followControl.h>
#include <iostream>

namespace DosClient
{

void FollowControl::setMoveK1(float valIn)
{
	m_k1 = valIn;
}

float FollowControl::getMoveK1() const
{
	return m_k1;
}

void FollowControl::setMoveK2(float valIn)
{
	m_k2 = valIn;
}

float FollowControl::getMoveK2() const
{
	return m_k2;
}

void FollowControl::setStopAreaFrac(float valIn)
{
	m_stopAreaFrac = valIn;
}

float FollowControl::getStopAreaFrac() const
{
	return m_stopAreaFrac;
}

void FollowControl::setForwardsFrac(float valIn)
{
	m_forwardsFrac = valIn;
}

float FollowControl::getForwardsFrac() const
{
	return m_forwardsFrac;
}

void FollowControl::setRotDXCutoff(float valIn)
{
	m_rotDXCutoff = valIn;
}

float FollowControl::getRotDXCutoff() const
{
	return m_rotDXCutoff;
}

void FollowControl::setKSwitchCutoff(float valIn)
{
	m_kSwitchCutoff = valIn;
}

float FollowControl::getKSwitchCutoff() const
{
	return m_kSwitchCutoff;
}

void FollowControl::setMaxMoveSpeed(float valIn)
{
	m_maxMoveSpeed = valIn;
}

float FollowControl::getMaxMoveSpeed() const
{
	return m_maxMoveSpeed;
}

void FollowControl::setRotBlend(float valIn)
{
	m_rotBlend = valIn;
}

float FollowControl::getRotBlend() const
{
	return m_rotBlend;
}

glm::vec3 
FollowControl::computeFollowFreeMove(const glm::vec2& imgSize, const glm::vec2& trackPt1, const glm::vec2& trackPt2)
{
	auto trackSize = trackPt2 - trackPt1;
	auto trackMidpt = ((trackPt1 + trackPt2) * 0.5f);
	auto trackArea = trackSize.x * trackSize.y;
	auto imgArea = imgSize.x * imgSize.y;
	glm::vec2 imgMidpt = imgSize * 0.5f;

	float forwardsAtten = 1.0f;
	if ((trackArea / imgArea) > getStopAreaFrac())
	{
		forwardsAtten = 0.0f;
	}

	glm::vec3 retVec(0, 0, m_maxMoveSpeed);
	float dx = trackMidpt.x - imgMidpt.x;
	float absDX = std::abs(dx);

	float realK = (absDX > m_kSwitchCutoff) ? m_k2 : m_k1;
	float rotVal = -realK * dx;
	if (absDX < m_rotDXCutoff)
	{
		rotVal = 0.0f;
	}

	if ((rotVal == 0.0f) && (forwardsAtten == 0.0f))
	{
		return glm::vec3(0);
	}

	retVec.x = interpScalar(retVec.x, rotVal, m_rotBlend);
	if (retVec.x < -1.0f) { retVec.x = -1.0f; }
	if (retVec.x > 1.0f) { retVec.x = 1.0f; }
	retVec.y = forwardsAtten * m_forwardsFrac;

	return retVec;
}

float FollowControl::interpScalar(float val1, float val2, float fraction) const
{
	return ((1.0f - fraction) * val1) + (fraction * val2);
}

} // end of namespace DosClient