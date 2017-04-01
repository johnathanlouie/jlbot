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
 * File:   worldmodel.cc
 * Author: Johnathan Louie
 *
 * Created on March 26, 2017, 5:52 PM
 */

#include "worldmodel.h"
#include <fstream>
#include <iostream>

namespace jlbot {

  ModelCoordinates::ModelCoordinates() {
  }

  ModelCoordinates::ModelCoordinates(int x, int y) {
    x_ = x;
    y_ = y;
  }

  int ModelCoordinates::GetX() {
    return x_;
  }

  int ModelCoordinates::GetY() {
    return y_;
  }

  bool ModelCoordinates::Equals(ModelCoordinates other) {
    return x_ == other.x_ && y_ == other.y_;
  }

  /* Initialize map to 0's, meaning all free space */
  void WorldModel::Empty() {
    for (int y = 0; y < kGridRows; y++) {
      for (int x = 0; x < kGridCols; x++) {
        SetEmpty(ModelCoordinates(x, y));
      }
    }
  }

  void WorldModel::SetEmpty(ModelCoordinates coordinates) {
    SetValue(coordinates, kEmpty);
  }

  void WorldModel::SetObstacle(ModelCoordinates coordinates) {
    SetValue(coordinates, kObstacle);
  }

  void WorldModel::SetPath(ModelCoordinates coordinates) {
    SetValue(coordinates, kPath);
  }

  void WorldModel::ReadMap(std::string filename) {
    std::cout << "Creating world model...." << std::endl;
    std::ifstream stream(filename);
    /* Read past first line */
    stream.getline(pnm_first_line_, 80);
    /* Read in width, height, maxVal */
    stream >> pnm_width_ >> pnm_height_ >> pnm_max_val_;
    model_height_ = pnm_height_ / kScaleMap;
    model_width_ = pnm_width_ / kScaleMap;
    /* Read in map; */
    for (int i = 0; i < pnm_height_; i++) {
      for (int j = 0; j < pnm_width_; j++) {
        char next_char;
        stream >> next_char;
        if (!next_char) {
          int y = i / kScaleMap;
          int x = j / kScaleMap;
          SetObstacle(ModelCoordinates(x, y));
        }
      }
    }
    std::cout << "World model complete." << std::endl;
    std::cout << "World dimensions (meters)" << std::endl;
    std::cout << " - Width: " << kWorldWidth << std::endl;
    std::cout << " - Height: " << kWorldHeight << std::endl;
    std::cout << "Model dimensions (pixels)" << std::endl;
    std::cout << " - Width: " << model_width_ << std::endl;
    std::cout << " - Height: " << model_height_ << std::endl;
    std::cout << "Model resolution (pixels/meter)" << std::endl;
    std::cout << " - Width: " << model_width_ / kWorldWidth << std::endl;
    std::cout << " - Height: " << model_height_ / kWorldHeight << std::endl;
  }

  WorldModel::WorldModel(std::string filename) {
    Empty();
    ReadMap(filename);
  }

  void WorldModel::Save(std::string filename) {
    std::cout << "Saving world model...." << std::endl;
    std::ofstream stream(filename);
    stream << pnm_first_line_ << std::endl;
    stream << model_width_ << " " << model_height_ << std::endl;
    stream << pnm_max_val_ << std::endl;
    for (int y = 0; y < model_height_; y++) {
      for (int x = 0; x < model_width_; x++) {
        ModelCoordinates position(x, y);
        unsigned char greyscale = 255;
        if (IsObstacle(position)) {
          greyscale = 180;
        } else if (IsEmpty(position)) {
          greyscale = 255;
        } else if (IsPath(position)) {
          greyscale = 0;
        }
        stream << greyscale;
      }
    }
    std::cout << "Save complete." << std::endl;
  }

  ModelCoordinates WorldModel::WorldToModel(WorldCoordinates world) {
    int x = (world.GetX() + kWorldWidth / 2) / kWorldWidth * model_width_;
    int y = (-world.GetY() + kWorldHeight / 2) / kWorldHeight * model_height_;
    return ModelCoordinates(x, y);
  }

  WorldCoordinates WorldModel::ModelToWorld(ModelCoordinates model) {
    double y = -model.GetY() * kWorldHeight / model_height_ + kWorldHeight / 2;
    double x = model.GetX() * kWorldWidth / model_width_ - kWorldWidth / 2;
    return WorldCoordinates(x, y);
  }

  int WorldModel::GetValue(ModelCoordinates coordinates) {
    return grid_map_[coordinates.GetY()][coordinates.GetX()];
  }

  void WorldModel::SetValue(ModelCoordinates coordinates, int value) {
    grid_map_[coordinates.GetY()][coordinates.GetX()] = value;
  }

  std::deque<ModelCoordinates> WorldModel::GetNeighbors(ModelCoordinates coordinates) {
    std::deque<ModelCoordinates> neighbors;
    int radius = 1;
    for (int vertical_offset = -radius; vertical_offset <= radius; vertical_offset++) {
      for (int horizontal_offset = -radius; horizontal_offset <= radius; horizontal_offset++) {
        int y = coordinates.GetY() + vertical_offset;
        int x = coordinates.GetX() + horizontal_offset;
        if (y >= 0 && x >= 0 && y < model_height_ && x < model_width_) {
          neighbors.push_back(ModelCoordinates(x, y));
        }
      }
    }
    return neighbors;
  }

  int WorldModel::GetHeight() {
    return model_height_;
  }

  int WorldModel::GetWidth() {
    return model_width_;
  }

  bool WorldModel::IsEmpty(ModelCoordinates coordinates) {
    return GetValue(coordinates) == kEmpty;
  }

  bool WorldModel::IsObstacle(ModelCoordinates coordinates) {
    return GetValue(coordinates) == kObstacle;
  }

  bool WorldModel::IsPath(ModelCoordinates coordinates) {
    return GetValue(coordinates) == kPath;
  }
} // namespace jlbot