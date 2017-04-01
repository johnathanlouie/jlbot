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
 * File:   actors.h
 * Author: Johnathan Louie
 *
 * Created on March 16, 2017, 7:39 PM
 */

#ifndef ACTORS_H
#define ACTORS_H

#include "misc.h"
#include "sensors.h"

namespace jlbot {

  class Vector {
  public:
    Vector();
    Vector(Radians direction, double magnitude);
    Radians GetDirection();
    double GetMagnitude();
    Vector Normalize(double magnitude);
    Vector Magnify(double factor);
    Vector Add(Vector other);
  private:
    WorldCoordinates coordinates_;
    static double ComputeX(Radians direction, double magnitude);
    static double ComputeY(Radians direction, double magnitude);
    static Radians ComputeDirection(WorldCoordinates coordinates);
    static double ComputeMagnitude(WorldCoordinates coordinates);
    Vector(WorldCoordinates coordindates);
  };

  class ObstacleField {
  public:
    static Vector GetVector(double distance, Radians direction);
  private:
    static double GetMagnitude(double distance);
  };

  class WaypointField {
  public:
    WaypointField();
    WaypointField(WorldCoordinates waypoint);
    Vector GetVector(WorldCoordinates current_position);
    bool AtWaypoint(WorldCoordinates current_position);
  private:
    WorldCoordinates waypoint_;
    double GetDistance(WorldCoordinates current_position);
  };

  class Act {
  public:
    Act(Robot *robot, Sense *sensors);
    void GoTo(WorldCoordinates waypoint);
  private:
    Robot *robot_;
    Sense *sense_;
    WaypointField waypoint_field_;
    Vector GetAttractionVector();
    Vector AvoidObstaclesGroup(double magnitude, double degrees1, double degrees2, double degrees3);
    Vector AvoidFrontObstacles();
    Vector AvoidLeftObstacles();
    Vector AvoidRightObstacles();
    Vector GetCombinedVector();
  };
} // namespace jlbot
#endif /* ACTORS_H */
