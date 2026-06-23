// ======================================================================================
// Copyright 2017 State Key Laboratory of Remote Sensing Science, 
// Institute of Remote Sensing Science and Engineering, Beijing Normal University

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ======================================================================================

#include "c2cdist.h"
#include <cmath>


void c2cdist::calCloud2CloudDist(Cloth           & cloth,
                                 csf::PointCloud & pc,
                                 std::vector<int>& groundIndexes,
                                 std::vector<int>& offGroundIndexes) {
    groundIndexes.resize(0);
    offGroundIndexes.resize(0);

    for (std::size_t i = 0; i < pc.size(); i++) {
        double pc_x = pc[i].x;
        double pc_z = pc[i].z;

        double deltaX = pc_x - cloth.origin_pos.f[0];
        double deltaZ = pc_z - cloth.origin_pos.f[2];

        int col0 = int(deltaX / cloth.step_x);
        int row0 = int(deltaZ / cloth.step_y);
        int col1 = col0 + 1;
        int row1 = row0;
        int col2 = col0 + 1;
        int row2 = row0 + 1;
        int col3 = col0;
        int row3 = row0 + 1;

        double subdeltaX = (deltaX - col0 * cloth.step_x) / cloth.step_x;
        double subdeltaZ = (deltaZ - row0 * cloth.step_y) / cloth.step_y;

        double fxy
            = cloth.getParticle(col0, row0)->pos.f[1] * (1 - subdeltaX) * (1 - subdeltaZ) +
              cloth.getParticle(col3, row3)->pos.f[1] * (1 - subdeltaX) * subdeltaZ +
              cloth.getParticle(col2, row2)->pos.f[1] * subdeltaX * subdeltaZ +
              cloth.getParticle(col1, row1)->pos.f[1] * subdeltaX * (1 - subdeltaZ);
        double height_var = fxy - pc[i].y;

        if (std::fabs(height_var) < class_threshold) {
            groundIndexes.push_back(i);
        } else {
            offGroundIndexes.push_back(i);
        }
    }
}


void c2cdist::calCloud2CloudDist_light(float origin_x,
                                       float origin_z,
                                       float step_x,
                                       float step_y,
                                       std::vector<float> particle_heights,
                                       int row_size,
                                       csf::PointCloud & pc,
                                       std::vector<int>& groundIndexes,
                                       std::vector<int>& offGroundIndexes) {
    groundIndexes.resize(0);
    offGroundIndexes.resize(0);

    for (std::size_t i = 0; i < pc.size(); i++) {
        double pc_x = pc[i].x;
        double pc_z = pc[i].z;

        double deltaX = pc_x - origin_x;
        double deltaZ = pc_z - origin_z;

        int col0 = int(deltaX / step_x);
        int row0 = int(deltaZ / step_y);
        int col1 = col0 + 1;
        int row1 = row0;
        int col2 = col0 + 1;
        int row2 = row0 + 1;
        int col3 = col0;
        int row3 = row0 + 1;

        double subdeltaX = (deltaX - col0 * step_x) / step_x;
        double subdeltaZ = (deltaZ - row0 * step_y) / step_y;

        double fxy
            = particle_heights[row0 * row_size + col0] * (1 - subdeltaX) * (1 - subdeltaZ) +
              particle_heights[row3 * row_size + col3] * (1 - subdeltaX) * subdeltaZ       +
              particle_heights[row2 * row_size + col2] * subdeltaX * subdeltaZ             + 
              particle_heights[row1 * row_size + col1] * subdeltaX * (1 - subdeltaZ);

        double height_var = fxy - pc[i].y;

        if (std::fabs(height_var) < class_threshold) {
            groundIndexes.push_back(i);
        } else {
            offGroundIndexes.push_back(i);
        }
    }
}
