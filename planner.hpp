#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <vector>
#include <cmath>

struct Pose
{
  double x;
  double y;
  double theta; // In degrees
};

struct Waypoint
{
  double x;
  double y;
};

class Planner
{
public:
  Planner(const std::vector<Waypoint> &waypoints);

  double calculateHeadingToWaypoint(const Pose &currentPose, int waypointIndex) const;
  double calculateDistanceToWaypoint(const Pose &currentPose, int waypointIndex) const;

private:
  std::vector<Waypoint> waypoints_;
};

// Extern declarations
extern Planner planner;
extern const std::vector<Waypoint> waypoints;
extern const int numWaypoints;
extern float stoppingDistances[];

#endif // PLANNER_HPP
