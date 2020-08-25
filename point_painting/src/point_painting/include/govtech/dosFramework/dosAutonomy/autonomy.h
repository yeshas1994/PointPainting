#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <chrono>
#include <unordered_map>
#include <string>

namespace DosClient
{

class AutonomyBase
{
public:
	AutonomyBase();
	virtual ~AutonomyBase() {}

	static std::chrono::milliseconds getTimeMS() {
		return std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch()
			);
	}

	// Sets the delta for each tick to process
	void setTickDelta(int valIn);
	int getTickDelta() const;
	// Call this to attempt to run for a tick if a tick delta has passed
	void process();

	// Basic Trajectory functions
	void addTrajectory(const std::string& nameIn, const std::vector<glm::vec3>& trajectoryIn);
	const std::vector<glm::vec3>& getTrajectory(const std::string& nameIn) const;
	bool hasTrajectory(const std::string& nameIn) const;
	void removeTrajectory(const std::string& nameIn);
	void clearTrajectoryMap();
	int getTrajectoryNum() const;
	std::unordered_map<std::string, std::vector<glm::vec3>>& getTrajectoryMap();
	void setActiveTrajectory(const std::string& nameIn);
	const std::string& getActiveTrajectory() const;
	void setActiveTrajectoryIdx(int valIn);
	void changeActiveTrajectoryIdx(int deltaIn);
	const int getActiveTrajectoryIdx() const;

protected:
	virtual void tick() {}

protected:
	std::chrono::milliseconds m_curTime;
	int m_tickDelta = 500; // In milliseconds
	std::unordered_map<std::string, std::vector<glm::vec3>> m_TrajStoreMap;
	std::string m_ActiveTraj;
	int m_ActiveTrajIdx = 0;
};

} // end of namespace DosClient