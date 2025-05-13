#include "Planner.hpp"

// Define the global waypoints list
const std::vector<Waypoint> waypoints = {
    {-4.0, -3.0}, // Start
    {-2.0, -1.0},
    {1.0, -1.0},
    {2.0, -3.0},
    {5.0, -3.0},
    {5.0, -2.0},
    {5.0, 3.0},
    {0.0, 3.0},
    {0.0, 0.0} // End
};
const int numWaypoints = 9;


// Define the global Planner using the waypoints
Planner planner(waypoints);

Planner::Planner(const std::vector<Waypoint> &waypoints)
    : waypoints_(waypoints) {}

double Planner::calculateHeadingToWaypoint(const Pose &currentPose, int waypointIndex) const
{
  if (waypointIndex < 0 || waypointIndex >= waypoints_.size())
    return 0.0;

  const Waypoint &wp = waypoints_[waypointIndex];
  double dx = wp.x - currentPose.x;
  double dy = wp.y - currentPose.y;
  double targetAngle = std::atan2(dy, dx) * 180.0 / M_PI; // radians to degrees

  double deltaTheta = targetAngle - currentPose.theta;
  while (deltaTheta > 180.0)
    deltaTheta -= 360.0;
  while (deltaTheta < -180.0)
    deltaTheta += 360.0;

  return deltaTheta;
}

double Planner::calculateDistanceToWaypoint(const Pose &currentPose, int waypointIndex) const
{
  if (waypointIndex < 0 || waypointIndex >= waypoints_.size())
    return 0.0;

  const Waypoint &wp = waypoints_[waypointIndex];
  double dx = wp.x - currentPose.x;
  double dy = wp.y - currentPose.y;
  return std::sqrt(dx * dx + dy * dy);
}
