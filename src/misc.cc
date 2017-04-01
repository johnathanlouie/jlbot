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
 * File:   misc.cc
 * Author: Johnathan Louie
 *
 * Created on March 17, 2017, 11:25 PM
 */

#include "misc.h"
#include <cmath>

namespace jlbot {

  Robot::Robot() {
    server_ = new PlayerCc::PlayerClient("localhost", 6665);
    pp_ = new PlayerCc::Position2dProxy(server_, 0);
    lp_ = new PlayerCc::LaserProxy(server_, 0);
    pp_->SetMotorEnable(true);
  }

  Robot::~Robot() {
    delete server_;
    delete pp_;
    delete lp_;
  }

  const WorldCoordinates WorldCoordinates::kOrigin(0, 0);

  WorldCoordinates Robot::GetGps() {
    return WorldCoordinates(pp_->GetXPos(), pp_->GetYPos());
  }

  double Robot::GetLaser(Radians direction) {
    int index = Degrees(direction).ToAtan2() + 90;
    return lp_->GetRange(index);
  }

  void Robot::Read() {
    server_->Read();
  }

  void Robot::Move(double longitudinal_speed, double yaw_speed) {
    pp_->SetSpeed(longitudinal_speed, yaw_speed);
  }

  Radians Robot::Facing() {
    return Radians(pp_->GetYaw());
  }

  WorldCoordinates::WorldCoordinates() {
  }

  WorldCoordinates::WorldCoordinates(double x, double y) {
    x_ = x;
    y_ = y;
  }

  WorldCoordinates WorldCoordinates::Add(WorldCoordinates other) {
    double x = other.x_ + x_;
    double y = other.y_ + y_;
    return WorldCoordinates(x, y);
  }

  double WorldCoordinates::GetX() {
    return x_;
  }

  double WorldCoordinates::GetY() {
    return y_;
  }

  double WorldCoordinates::Distance(WorldCoordinates other) {
    double x = std::pow(x_ - other.x_, 2);
    double y = std::pow(y_ - other.y_, 2);
    return std::sqrt(x + y);
  }

  std::string WorldCoordinates::ToString() {
    std::ostringstream stream;
    stream << "(" << x_ << ", " << y_ << ")";
    return stream.str();
  }

  Radians::Radians() {
    radians_ = 0;
  }

  Radians::Radians(double radians) {
    radians_ = Normalize(radians);
  }

  Radians::Radians(WorldCoordinates reference, WorldCoordinates target) {
    double x = target.GetX() - reference.GetX();
    double y = target.GetY() - reference.GetY();
    radians_ = Normalize(std::atan2(y, x));
  }

  double Radians::Normalize(double radians) {
    radians = std::fmod(radians, 2 * M_PI);
    if (radians < 0) {
      radians += 2 * M_PI;
    }
    return radians;
  }

  Radians Radians::Flip() {
    return Radians(radians_ + M_PI);
  }

  /* returns angle as radians in domain 0 to 2 pi */
  double Radians::ToDouble() {
    return radians_;
  }

  /* returns angle as radians in domain -pi to +pi*/
  double Radians::ToAtan2() {
    if (radians_ <= M_PI) {
      return radians_;
    }
    return radians_ - 2 * M_PI;
  }

  double Radians::Difference(Radians target) {
    double angle = target.radians_ - radians_;
    if (std::abs(angle) > M_PI) {
      if (angle > 0) {
        angle -= 2 * M_PI;
      } else {
        angle += 2 * M_PI;
      }
    }
    return angle;
  }

  Degrees::Degrees() {
  }

  Degrees::Degrees(double degrees) {
    double radians = DegreesToRadians(degrees);
    angle_ = Radians(radians);
  }

  Degrees::Degrees(Radians angle) {
    angle_ = angle;
  }

  Degrees::Degrees(WorldCoordinates reference, WorldCoordinates target) {
    angle_ = Radians(reference, target);
  }

  double Degrees::RadiansToDegrees(double radians) {
    return radians / M_PI * 180;
  }

  double Degrees::DegreesToRadians(double degrees) {
    return degrees / 180 * M_PI;
  }

  double Degrees::ToDouble() {
    return RadiansToDegrees(angle_.ToDouble());
  }

  double Degrees::ToAtan2() {
    double a = ToDouble();
    if (a <= 180) {
      return a;
    }
    return a - 360;
  }

  double Degrees::Difference(Degrees target) {
    return RadiansToDegrees(angle_.Difference(target.angle_));
  }

  Degrees Degrees::Flip() {
    return Degrees(angle_.Flip());
  }

  Radians Degrees::ToRadians() {
    return angle_;
  }
} // namespace jlbot