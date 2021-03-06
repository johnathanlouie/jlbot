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
 * File:   sensors.cc
 * Author: Johnathan Louie
 *
 * Created on March 16, 2017, 7:38 PM
 */

#include "sensors.h"

namespace jlbot {

  Sense::Sense(Robot *robot) {
    robot_ = robot;
  }

  WorldCoordinates Sense::GetCurrentPosition() {
    return robot_->GetGps();
  }

  double Sense::GetRange(Radians direction) {
    robot_->GetLaser(direction);
  }
} // namespace jlbot