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

// Corresponds to the number of waypoints
float stoppingDistances[] = {
    0.0, // waypoint 0
    0.0, // waypoint 1
    0.0, // waypoint 2
    0.0, // waypoint 3
    1.0, // waypoint 4 --> early stop at 1 foot
    0.0, // waypoint 5
    2.0, // waypoint 6 --> early stop at 1 foot
    3.0, // waypoint 7 --> early stop at 2 feet
    0.0  // waypoint 8
};

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
