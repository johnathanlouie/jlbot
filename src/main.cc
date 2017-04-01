/*
 * Copyright (C) 2017 jlouie
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
 * File:   main.cc
 * Author: Johnathan Louie
 *
 * Created on March 16, 2017, 7:39 PM
 */

#include <iostream>
#include <string>
#include <libplayerc++/playerc++.h>
#include "actors.h"
#include "misc.h"
#include "planners.h"
#include "sensors.h"

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "USAGE: jlbot x y" << std::endl;
    return EXIT_FAILURE;
  }
  try {
    std::cout << "Connecting to player server...." << std::endl;
    jlbot::Robot *robot = new jlbot::Robot();
    double goal_x = strtod(argv[1], NULL);
    double goal_y = strtod(argv[2], NULL);
    jlbot::WorldCoordinates goal(goal_x, goal_y);
    std::cout << "Goal set to " << goal.ToString() << "." << std::endl;
    jlbot::Sense *sensors = new jlbot::Sense(robot);
    robot->Read();
    jlbot::WorldCoordinates current_position = sensors->GetCurrentPosition();
    jlbot::Navigator navigator(current_position, goal);
    if (!navigator.HasPath()) {
      return EXIT_SUCCESS;
    }
    jlbot::Pilot pilot = navigator.GetPilot();
    jlbot::Act act(robot, sensors);
    while (pilot.HasObjectives()) {
      jlbot::WorldCoordinates waypoint = pilot.GetNextObjective();
      act.GoTo(waypoint);
      pilot.ReachedObjective();
    }
    std::cout << "Robot reached the goal." << std::endl;
  } catch (PlayerCc::PlayerError &error) {
    std::cerr << error << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
