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
 * File:   worldmodel.h
 * Author: Johnathan Louie
 *
 * Created on March 26, 2017, 5:52 PM
 */

#ifndef WORLDMODEL_H
#define WORLDMODEL_H

#include <climits>
#include <deque>
#include <string>
#include "misc.h"

namespace jlbot {

  class ModelCoordinates {
  public:
    ModelCoordinates();
    ModelCoordinates(int x, int y);
    int GetX();
    int GetY();
    bool Equals(ModelCoordinates other);
  private:
    int x_;
    int y_;
  };

  class WorldModel {
  public:
    static const int kEmpty = INT_MIN;
    static const int kObstacle = INT_MIN + 1;
    static const int kPath = INT_MIN + 2;
    WorldModel(std::string filename);
    ModelCoordinates WorldToModel(WorldCoordinates world);
    WorldCoordinates ModelToWorld(ModelCoordinates model);
    int GetValue(ModelCoordinates coordinates);
    void SetValue(ModelCoordinates coordinates, int value);
    std::deque<ModelCoordinates> GetNeighbors(ModelCoordinates coordinates);
    void Save(std::string filename);
    int GetHeight();
    int GetWidth();
    bool IsEmpty(ModelCoordinates coordinates);
    bool IsObstacle(ModelCoordinates coordinates);
    bool IsPath(ModelCoordinates coordinates);
    void SetEmpty(ModelCoordinates coordinates);
    void SetObstacle(ModelCoordinates coordinates);
    void SetPath(ModelCoordinates coordinates);
  private:
    static const int kScaleMap = 2;
    static const int kGridRows = 1000;
    static const int kGridCols = 1000;
    const double kWorldWidth = 40;
    const double kWorldHeight = 18;
    char pnm_first_line_[80];
    int pnm_width_;
    int pnm_height_;
    int pnm_max_val_;
    int model_height_;
    int model_width_;
    int grid_map_[kGridRows][kGridCols];
    void Empty();
    void ReadMap(std::string filename);
  };
} // namespace jlbot
#endif /* WORLDMODEL_H */
