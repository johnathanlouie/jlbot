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
 * File:   misc.h
 * Author: Johnathan Louie
 *
 * Created on March 17, 2017, 11:25 PM
 */

#ifndef MISC_H
#define MISC_H

#include <string>
#include <libplayerc++/playerc++.h>

namespace jlbot {

  class WorldCoordinates {
  public:
    const static WorldCoordinates kOrigin;
    WorldCoordinates();
    WorldCoordinates(double x, double y);
    double GetX();
    double GetY();
    WorldCoordinates Add(WorldCoordinates other);
    double Distance(WorldCoordinates other);
    std::string ToString();
  private:
    double x_;
    double y_;
  };

  class Radians {
  public:
    Radians();
    Radians(double radians);
    Radians(WorldCoordinates reference, WorldCoordinates target);
    Radians Flip();

    /* returns angle as radians in domain 0 to 2 pi */
    double ToDouble();

    /* returns angle as radians in domain -pi to +pi*/
    double ToAtan2();
    double Difference(Radians target);
  private:
    double radians_;
    static double Normalize(double radians);
  };

  class Degrees {
  public:
    Degrees();
    Degrees(double degrees);
    Degrees(Radians angle);
    Degrees(WorldCoordinates reference, WorldCoordinates target);
    static double RadiansToDegrees(double radians);
    static double DegreesToRadians(double degrees);
    double ToDouble();
    double ToAtan2();
    double Difference(Degrees target);
    Degrees Flip();
    Radians ToRadians();
  private:
    Radians angle_;
  };

  class Robot {
  public:
    Robot();
    ~Robot();
    WorldCoordinates GetGps();
    double GetLaser(Radians direction);
    void Read();
    void Move(double longitudinal_speed, double yaw_speed);
    Radians Facing();
  private:
    PlayerCc::PlayerClient *server_;
    PlayerCc::Position2dProxy *pp_;
    PlayerCc::LaserProxy *lp_;
  };
} // namespace jlbot
#endif /* MISC_H */
