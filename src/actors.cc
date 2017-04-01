/*
 * Copyright (C) 2017 Johnathan Louie
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

/*
 * File:   actors.cc
 * Author: Johnathan Louie
 *
 * Created on March 16, 2017, 7:39 PM
 */

#include "actors.h"
#include <cmath>
#include <deque>
#include <iostream>
#include <libplayerc++/playerc++.h>

namespace jlbot {

  Vector::Vector() : coordinates_(0, 0) {
  }

  Vector::Vector(Radians direction, double magnitude) {
    double x = ComputeX(direction, magnitude);
    double y = ComputeY(direction, magnitude);
    coordinates_ = WorldCoordinates(x, y);
  }

  Vector::Vector(WorldCoordinates coordindates) {
    coordinates_ = coordindates;
  }

  double Vector::ComputeX(Radians direction, double magnitude) {
    double radians = direction.ToDouble();
    return magnitude * std::cos(radians);
  }

  double Vector::ComputeY(Radians direction, double magnitude) {
    double radians = direction.ToDouble();
    return magnitude * std::sin(radians);
  }

  Radians Vector::ComputeDirection(WorldCoordinates coordinates) {
    return Radians(WorldCoordinates::kOrigin, coordinates);
  }

  double Vector::ComputeMagnitude(WorldCoordinates coordinates) {
    return coordinates.Distance(WorldCoordinates::kOrigin);
  }

  Radians Vector::GetDirection() {
    return ComputeDirection(coordinates_);
  }

  double Vector::GetMagnitude() {
    return ComputeMagnitude(coordinates_);
  }

  Vector Vector::Normalize(double magnitude) {
    return Vector(GetDirection(), magnitude);
  }

  Vector Vector::Magnify(double factor) {
    Radians direction = GetDirection();
    double magnitude = GetMagnitude() * factor;
    return Vector(direction, magnitude);
  }

  Vector Vector::Add(Vector other) {
    WorldCoordinates sum = coordinates_.Add(other.coordinates_);
    return Vector(sum);
  }

  double ObstacleField::GetMagnitude(double distance) {
    double preferred_distance = 0.3;
    return distance < preferred_distance ? 1 : 0;
  }

  Vector ObstacleField::GetVector(double distance, Radians direction) {
    Radians flipped_direction = direction.Flip();
    double magnitude = GetMagnitude(distance);
    return Vector(flipped_direction, magnitude);
  }

  WaypointField::WaypointField() {
  }

  WaypointField::WaypointField(WorldCoordinates waypoint) {
    waypoint_ = waypoint;
  }

  Vector WaypointField::GetVector(WorldCoordinates current_position) {
    double magnitude = 1;
    Radians direction(current_position, waypoint_);
    return Vector(direction, magnitude);
  }

  double WaypointField::GetDistance(WorldCoordinates current_position) {
    return current_position.Distance(waypoint_);
  }

  bool WaypointField::AtWaypoint(WorldCoordinates current_position) {
    double distance_from_waypoint = GetDistance(current_position);
    return distance_from_waypoint < 0.4;
  }

  Act::Act(Robot *robot, Sense *sensors) {
    robot_ = robot;
    sense_ = sensors;
  }

  void Act::GoTo(WorldCoordinates waypoint) {
    std::cout << "Going to waypoint " << waypoint.ToString() << "...." << std::endl;
    waypoint_field_ = WaypointField(waypoint);
    robot_->Read();
    while (!waypoint_field_.AtWaypoint(sense_->GetCurrentPosition())) {
      Vector final_field = GetCombinedVector();
      Radians desired_direction = final_field.GetDirection();
      Radians current_direction = robot_->Facing();
      double turn_rate = current_direction.Difference(desired_direction);
      double max_turn_rate = M_PI / 3;
      double turn_speed = PlayerCc::limit(std::abs(turn_rate), 0.0, max_turn_rate);
      double longitudinal_speed = 4 * std::pow(max_turn_rate - turn_speed, 2);
      longitudinal_speed = PlayerCc::limit(longitudinal_speed, 0.0, 4.0);
      robot_->Move(longitudinal_speed, turn_rate);
      robot_->Read();
    }
    std::cout << "Reached waypoint." << std::endl;
  }

  Vector Act::GetAttractionVector() {
    WorldCoordinates current_location = sense_->GetCurrentPosition();
    return waypoint_field_.GetVector(current_location);
  }

  Vector Act::AvoidObstaclesGroup(double magnitude, double degrees1, double degrees2, double degrees3) {
    std::deque<Degrees> degrees = {Degrees(degrees1), Degrees(degrees2), Degrees(degrees3)};
    Vector combined;
    for (Degrees i : degrees) {
      Radians j = i.ToRadians();
      Vector temp = ObstacleField::GetVector(sense_->GetRange(j), j);
      combined.Add(temp);
    }
    if (combined.GetMagnitude() >= 1) {
      combined = combined.Normalize(magnitude);
    }
    return combined;
  }

  Vector Act::AvoidFrontObstacles() {
    return AvoidObstaclesGroup(1, 0, -10, 10);
  }

  Vector Act::AvoidLeftObstacles() {
    return AvoidObstaclesGroup(1, -45, -60, -89);
  }

  Vector Act::AvoidRightObstacles() {
    return AvoidObstaclesGroup(1, 45, 60, 89);
  }

  Vector Act::GetCombinedVector() {
    Vector combined;
    combined = combined.Add(GetAttractionVector());
    combined = combined.Add(AvoidFrontObstacles());
    combined = combined.Add(AvoidRightObstacles());
    combined = combined.Add(AvoidLeftObstacles());
    return combined;
  }
} // namespace jlbot