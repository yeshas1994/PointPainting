#include <dosAutonomy/autonomy.h>

namespace DosClient
{

AutonomyBase::AutonomyBase()
{
	m_curTime = getTimeMS();
}

void AutonomyBase::setTickDelta(int valIn)
{
	m_tickDelta = valIn;
}

int AutonomyBase::getTickDelta() const
{
	return m_tickDelta;
}

void AutonomyBase::process()
{
	auto cTime = getTimeMS();
	auto diffTime = cTime.count() - m_curTime.count();
	if (diffTime > m_tickDelta)
	{
		m_curTime = cTime;
		tick();
	}
}

void AutonomyBase::addTrajectory(const std::string& nameIn, const std::vector<glm::vec3>& trajectoryIn)
{
}

const std::vector<glm::vec3>& AutonomyBase::getTrajectory(const std::string& nameIn) const
{
	static std::vector<glm::vec3> emptyList;
	auto fIter = m_TrajStoreMap.find(nameIn);
	if (fIter == m_TrajStoreMap.end())
	{
		return emptyList;
	}
	return fIter->second;
}

bool AutonomyBase::hasTrajectory(const std::string& nameIn) const
{
	return m_TrajStoreMap.count(nameIn) > 0;
}

void AutonomyBase::removeTrajectory(const std::string& nameIn)
{
	auto fIter = m_TrajStoreMap.find(nameIn);
	if (fIter != m_TrajStoreMap.end())
	{
		m_TrajStoreMap.erase(fIter);
	}
}

void AutonomyBase::clearTrajectoryMap()
{
	m_TrajStoreMap.clear();
}

int AutonomyBase::getTrajectoryNum() const
{
	return static_cast<int>(m_TrajStoreMap.size());
}

std::unordered_map<std::string, std::vector<glm::vec3>>& AutonomyBase::getTrajectoryMap()
{
	return m_TrajStoreMap;
}

void AutonomyBase::setActiveTrajectory(const std::string& nameIn)
{
	m_ActiveTraj = nameIn;
}

const std::string& AutonomyBase::getActiveTrajectory() const
{
	return m_ActiveTraj;
}

void AutonomyBase::setActiveTrajectoryIdx(int valIn)
{
	m_ActiveTrajIdx = valIn;
}

void AutonomyBase::changeActiveTrajectoryIdx(int deltaIn)
{
	m_ActiveTrajIdx += deltaIn;
}

const int AutonomyBase::getActiveTrajectoryIdx() const
{
	return m_ActiveTrajIdx;
}

} // end of namespace DosClient