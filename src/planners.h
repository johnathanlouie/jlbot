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
 * File:   planners.h
 * Author: Johnathan Louie
 *
 * Created on March 16, 2017, 7:38 PM
 */

#ifndef PLANNERS_H
#define PLANNERS_H

#include <deque>
#include "misc.h"
#include "worldmodel.h"

namespace jlbot {

  class Pilot {
  public:
    Pilot();
    Pilot(std::deque<WorldCoordinates> path);
    void ReachedObjective();
    bool HasObjectives();
    WorldCoordinates GetNextObjective();
  private:
    std::deque<WorldCoordinates> path_;
    int current_objective_;
  };

  class Navigator {
  public:
    void TracePath(std::deque<ModelCoordinates> path, WorldModel *world_model);
    Pilot GetPilot();
    bool HasPath();
    Navigator(WorldCoordinates start, WorldCoordinates goal);
  private:
    WorldModel model_;
    bool has_path_;
    std::deque<WorldCoordinates> path_;
    void GrowObstacles(int thickness);
    std::deque<ModelCoordinates> Wavefront(ModelCoordinates start, ModelCoordinates goal);
    int PropagateWave(ModelCoordinates start, ModelCoordinates goal);
    std::deque<ModelCoordinates> ExtractPath(ModelCoordinates start, int count);
    std::deque<ModelCoordinates> RelaxPath(std::deque<ModelCoordinates> path);
    std::deque<ModelCoordinates> GetStraightLinePath(ModelCoordinates a, ModelCoordinates b);
    bool IsClear(std::deque<ModelCoordinates> path);
    std::deque<WorldCoordinates> ModelToWorld(std::deque<ModelCoordinates> model_path);
  };
} // namespace jlbot
#endif /* PLANNERS_H */
