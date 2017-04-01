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
 * File:   planners.cc
 * Author: Johnathan Louie
 *
 * Created on March 16, 2017, 7:38 PM
 */

#include "planners.h"
#include <cmath>
#include <iostream>

namespace jlbot {

  Navigator::Navigator(WorldCoordinates start, WorldCoordinates goal) : model_("hospital_section.pnm") {
    model_.Save("0_scaled.pnm");
    WorldModel *full_path_model = new WorldModel(model_);
    WorldModel *relaxed_path_model = new WorldModel(model_);
    GrowObstacles(4);
    model_.Save("1_grow_obstacles.pnm");
    ModelCoordinates begin = model_.WorldToModel(start);
    ModelCoordinates end = model_.WorldToModel(goal);
    std::deque<ModelCoordinates> temp_path = Wavefront(begin, end);
    TracePath(temp_path, full_path_model);
    full_path_model->Save("2_full_path.pnm");
    temp_path = RelaxPath(temp_path);
    TracePath(temp_path, relaxed_path_model);
    relaxed_path_model->Save("3_relaxed_path.pnm");
    path_ = ModelToWorld(temp_path);
    path_.push_front(start);
    path_.push_back(goal);
  }

  void Navigator::GrowObstacles(int thickness) {
    std::cout << "Growing obstacles by " << thickness << " pixels...." << std::endl;
    static const int kNewObstacle = -3;
    for (int i = 0; i < thickness; i++) {
      for (int y = 0; y < model_.GetHeight(); y++) {
        for (int x = 0; x < model_.GetWidth(); x++) {
          ModelCoordinates current(x, y);
          if (model_.IsObstacle(current)) {
            for (ModelCoordinates neighbor : model_.GetNeighbors(current)) {
              if (model_.IsEmpty(neighbor)) {
                model_.SetValue(neighbor, kNewObstacle);
              }
            }
          }
        }
      }
      for (int y = 0; y < model_.GetHeight(); y++) {
        for (int x = 0; x < model_.GetWidth(); x++) {
          ModelCoordinates current(x, y);
          if (model_.GetValue(current) == kNewObstacle) {
            model_.SetObstacle(current);
          }
        }
      }
    }
  }

  int Navigator::PropagateWave(ModelCoordinates start, ModelCoordinates goal) {
    std::cout << "Propagating wave...." << std::endl;
    int count = 0;
    model_.SetValue(goal, count);
    if (start.Equals(goal)) {
      return count;
    }
    std::deque<ModelCoordinates> fringe;
    fringe.push_back(goal);
    while (!fringe.empty()) {
      count++;
      std::deque<ModelCoordinates> new_fringe;
      for (ModelCoordinates fringe_element : fringe) {
        std::deque<ModelCoordinates> neighbors = model_.GetNeighbors(fringe_element);
        for (ModelCoordinates neighbor : neighbors) {
          if (model_.IsEmpty(neighbor)) {
            model_.SetValue(neighbor, count);
            if (start.Equals(fringe_element)) {
              return count;
            }
            new_fringe.push_back(neighbor);
          }
        }
      }
      fringe = new_fringe;
    }
    return -1;
  }

  std::deque<ModelCoordinates> Navigator::GetStraightLinePath(ModelCoordinates a, ModelCoordinates b) {
    std::deque<ModelCoordinates> path;
    if (a.GetX() == b.GetX()) {
      int y_min = std::min(a.GetY(), b.GetY());
      int y_max = std::max(a.GetY(), b.GetY());
      for (int y = y_min; y <= y_max; y++) {
        path.push_back(ModelCoordinates(a.GetX(), y));
      }
    } else {
      double x1 = a.GetX();
      double x2 = b.GetX();
      double y1 = a.GetY();
      double y2 = b.GetY();
      double m = (y1 - y2) / (x1 - x2);
      double b = -x1 * m + y1;
      double x_min = std::min(x1, x2);
      double x_max = std::max(x1, x2);
      for (int x = x_min; x <= x_max; x++) {
        double y = m * x + b;
        y = std::round(y);
        path.push_back(ModelCoordinates(x, y));
      }
    }
    return path;
  }

  bool Navigator::IsClear(std::deque<ModelCoordinates> path) {
    for (ModelCoordinates i : path) {
      if (model_.IsObstacle(i)) {
        return false;
      }
    }
    return true;
  }

  std::deque<ModelCoordinates> Navigator::RelaxPath(std::deque<ModelCoordinates> path) {
    std::cout << "Relaxing path...." << std::endl;
    if (path.size() < 3) {
      return path;
    }
    int last = path.size() - 1;
    std::deque<ModelCoordinates> relaxed_path;
    relaxed_path.push_back(path[0]);
    for (int reference = 0, clear = 1, test = 1; test <= last - 1;) {
      std::deque<ModelCoordinates> straight_line = GetStraightLinePath(path[reference], path[clear]);
      if (IsClear(straight_line)) {
        clear = test;
        test++;
      } else {
        relaxed_path.push_back(path[clear]);
        reference = clear;
      }
    }
    std::cout << "Relaxed path has " << relaxed_path.size() << " waypoints." << std::endl;
    return relaxed_path;
  }

  std::deque<ModelCoordinates> Navigator::ExtractPath(ModelCoordinates start, int count) {
    ModelCoordinates current = start;
    std::deque<ModelCoordinates> path;
    path.push_back(start);
    for (int i = count; i > 0; i--) {
      for (ModelCoordinates neighbor : model_.GetNeighbors(current)) {
        if (model_.GetValue(neighbor) == i - 1) {
          current = neighbor;
          path.push_back(neighbor);
          break;
        }
      }
    }
    std::cout << "Wavefront generated a path with " << path.size() << " waypoints." << std::endl;
    return path;
  }

  void Navigator::TracePath(std::deque<ModelCoordinates> path, WorldModel *model) {
    for (ModelCoordinates i : path) {
      model->SetPath(i);
    }
    std::cout << "Printed path to the world model." << std::endl;
  }

  std::deque<WorldCoordinates> Navigator::ModelToWorld(std::deque<ModelCoordinates> model_path) {
    std::deque<WorldCoordinates> world_path;
    for (ModelCoordinates i : model_path) {
      world_path.push_back(model_.ModelToWorld(i));
    }
    return world_path;
  }

  std::deque<ModelCoordinates> Navigator::Wavefront(ModelCoordinates start, ModelCoordinates goal) {
    int count = PropagateWave(start, goal);
    std::deque<ModelCoordinates> path;
    if (count == -1) {
      std::cout << "Goal is unreachable." << std::endl;
      has_path_ = false;
    } else {
      std::cout << "Found a path to the goal." << std::endl;
      has_path_ = true;
      path = ExtractPath(start, count);
    }
    return path;
  }

  Pilot Navigator::GetPilot() {
    return Pilot(path_);
  }

  bool Navigator::HasPath() {
    return has_path_;
  }

  Pilot::Pilot(std::deque<WorldCoordinates> path) {
    path_ = path;
    current_objective_ = 0;
  }

  void Pilot::ReachedObjective() {
    current_objective_++;
  }

  bool Pilot::HasObjectives() {
    return path_.size() > current_objective_;
  }

  WorldCoordinates Pilot::GetNextObjective() {
    return path_[current_objective_];
  }
} // namespace jlbot