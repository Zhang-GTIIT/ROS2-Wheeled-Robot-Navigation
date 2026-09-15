/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Simbe Robotics, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Macenski (steven.macenski@simberobotics.com)
 *********************************************************************/

#include <memory>
#include <unordered_map>
#include <string>
#include <vector>

#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"

namespace volume_grid
{

/*****************************************************************************/
// 辅助函数：判断点是否在多边形内 (射线法)
bool SpatioTemporalVoxelGrid::IsPointInsidePolygon(
  double x, double y, 
  const std::vector<geometry_msgs::msg::Point>& polygon) const
{
  if (polygon.size() < 3) return false;
  bool inside = false;
  size_t n = polygon.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    if (((polygon[i].y > y) != (polygon[j].y > y)) &&
        (x < (polygon[j].x - polygon[i].x) * (y - polygon[i].y) / 
             (polygon[j].y - polygon[i].y) + polygon[i].x)) {
      inside = !inside;
    }
  }
  return inside;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::ClearFootprintVoxels(
  const std::vector<geometry_msgs::msg::Point>& footprint)
/*****************************************************************************/
{
  if (footprint.empty()) return;

  boost::unique_lock<boost::mutex> lock(_grid_lock);

  // 1. 计算 Footprint 的包围盒 (Bounding Box)
  // 这样我们只需要遍历一小块区域，而不是整个地图
  double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
  for (const auto& pt : footprint) {
    if (pt.x < min_x) min_x = pt.x;
    if (pt.x > max_x) max_x = pt.x;
    if (pt.y < min_y) min_y = pt.y;
    if (pt.y > max_y) max_y = pt.y;
  }

  // 2. 将包围盒的世界坐标转换为 Grid 索引坐标
  // 注意：OpenVDB 的索引是整数。需要考虑 voxel_size 转换。
  // WorldToIndex 返回的是 Vec3d，我们需要取整得到范围。
  openvdb::Vec3d min_idx_d = this->WorldToIndex(openvdb::Vec3d(min_x, min_y, 0));
  openvdb::Vec3d max_idx_d = this->WorldToIndex(openvdb::Vec3d(max_x, max_y, 0));

  // 为了保险起见，范围稍微扩大一点点，确保覆盖边缘
  int start_x = std::floor(min_idx_d.x()) - 1;
  int end_x   = std::ceil(max_idx_d.x()) + 1;
  int start_y = std::floor(min_idx_d.y()) - 1;
  int end_y   = std::ceil(max_idx_d.y()) + 1;
  
  // Z轴范围：如果不限制高度，需要遍历整个 Grid 的有效 Z 范围
  // 或者根据传感器配置的 min_z / max_z 来设定
  // 这里假设我们需要清除 footprint 上方一定高度内的障碍物
  // 获取活跃体素的迭代器太慢，我们直接利用 OpenVDB 的稀疏特性访问
  
  openvdb::DoubleGrid::Accessor accessor = _grid->getAccessor();

  // 3. 遍历包围盒内的 X, Y
  for (int ix = start_x; ix <= end_x; ++ix) {
    for (int iy = start_y; iy <= end_y; ++iy) {
      
      // 先计算当前索引对应的世界坐标中心 (只算 X, Y)
      // 这里的 Z 填 0 即可，因为 IsPointInsidePolygon 只看 XY
      openvdb::Coord temp_coord(ix, iy, 0);
      openvdb::Vec3d world_pos = this->IndexToWorld(temp_coord);

      // 4. 精确几何判断：该体素列是否在 Footprint 内
      if (IsPointInsidePolygon(world_pos.x(), world_pos.y(), footprint)) {
        
        // 5. 如果在 Footprint 内，清除该 (x, y) 下的所有 Z
        // OpenVDB 并没有直接提供 "清除一列" 的 API，我们需要遍历 Z
        // 为了效率，我们可以只清除“有值”的体素。
        // 但由于不知道 Z 的范围，我们可以利用 OpenVDB 的迭代器，或者简单地在一个合理范围内循环
        // 方法 A (暴力循环 Z): 适合 Z 范围不大的情况
        // 方法 B (利用 Tree 结构): 更复杂但更高效
        
        // 这里采用一种折中且安全的方法：
        // 假设障碍物高度一般在 -2m 到 5m 之间 (根据 voxel_size 换算索引)
        // 你可以根据实际参数调整这个 Z 范围
        int z_min_idx = -100; // 举例
        int z_max_idx = 100;  // 举例
        
        // 更好的方式：只查找存在的 voxel。
        // 但 OpenVDB 对随机访问 (ix, iy, z) 是 O(1) 的，所以直接循环 Z 并不慢，只要它不是无限大。
        // 我们可以复用 Layer 参数中的 min_z 和 max_z 对应的索引。
        
        // 为了演示代码简洁，这里假设一个足够大的索引范围，或者利用 accessor.probe
        for (int iz = z_min_idx; iz <= z_max_idx; ++iz) {
             openvdb::Coord pt(ix, iy, iz);
             // 如果该体素是激活的 (有障碍物)
             if (accessor.isValueOn(pt)) {
                 accessor.setValueOff(pt, _background_value);
             }
        }
      }
    }
  }
  // 清理内存碎片
  _grid->pruneGrid();
}

/*****************************************************************************/
SpatioTemporalVoxelGrid::SpatioTemporalVoxelGrid(
  rclcpp::Clock::SharedPtr clock,
  const float & voxel_size, const double & background_value,
  const int & decay_model, const double & voxel_decay, const bool & pub_voxels)
: _clock(clock), _decay_model(decay_model), _background_value(background_value),
  _voxel_size(voxel_size), _voxel_decay(voxel_decay), _pub_voxels(pub_voxels),
  _grid_points(std::make_unique<std::vector<geometry_msgs::msg::Point32>>()),
  _cost_map(new std::unordered_map<occupany_cell, uint>)
/*****************************************************************************/
{
  this->InitializeGrid();
}

/*****************************************************************************/
SpatioTemporalVoxelGrid::~SpatioTemporalVoxelGrid(void)
/*****************************************************************************/
{
  // pcl pointclouds free themselves
  if (_cost_map) {
    delete _cost_map;
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::InitializeGrid(void)
/*****************************************************************************/
{
  // initialize the OpenVDB Grid volume
  openvdb::initialize();

  // make it default to background value
  _grid = openvdb::DoubleGrid::create(_background_value);

  // setup scale and tranform
  openvdb::Mat4d m = openvdb::Mat4d::identity();
  m.preScale(openvdb::Vec3d(_voxel_size, _voxel_size, _voxel_size));
  m.preTranslate(openvdb::Vec3d(0, 0, 0));
  m.preRotate(openvdb::math::Z_AXIS, 0);

  // setup transform and other metadata
  _grid->setTransform(openvdb::math::Transform::createLinearTransform(m));
  _grid->setName("SpatioTemporalVoxelLayer");
  _grid->insertMeta("Voxel Size", openvdb::FloatMetadata(_voxel_size));
  _grid->setGridClass(openvdb::GRID_LEVEL_SET);
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::ClearFrustums(
  const std::vector<observation::MeasurementReading> & clearing_readings,
  std::unordered_set<occupany_cell> & cleared_cells)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  // accelerate the decay of voxels interior to the frustum
  if (this->IsGridEmpty()) {
    _grid_points->clear();
    _cost_map->clear();
    return;
  }

  _grid_points->clear();
  _cost_map->clear();

  std::vector<frustum_model> obs_frustums;

  if (clearing_readings.size() == 0) {
    TemporalClearAndGenerateCostmap(obs_frustums, cleared_cells);
    return;
  }

  obs_frustums.reserve(clearing_readings.size());

  std::vector<observation::MeasurementReading>::const_iterator it =
    clearing_readings.begin();
  for (; it != clearing_readings.end(); ++it) {
    geometry::Frustum * frustum = nullptr;
    if (it->_model_type == DEPTH_CAMERA) {
      frustum = new geometry::DepthCameraFrustum(
        it->_vertical_fov_in_rad,
        it->_horizontal_fov_in_rad, it->_min_z_in_m, it->_max_z_in_m);
    } else if (it->_model_type == THREE_DIMENSIONAL_LIDAR) {
      frustum = new geometry::ThreeDimensionalLidarFrustum(
        it->_vertical_fov_in_rad, it->_vertical_fov_padding_in_m,
        it->_horizontal_fov_in_rad, it->_min_z_in_m, it->_max_z_in_m);
    } else {
      // add else if statement for each implemented model
      delete frustum;
      continue;
    }

    frustum->SetPosition(it->_origin);
    frustum->SetOrientation(it->_orientation);
    frustum->TransformModel();
    obs_frustums.emplace_back(frustum, it->_decay_acceleration);
  }
  TemporalClearAndGenerateCostmap(obs_frustums, cleared_cells);
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::TemporalClearAndGenerateCostmap(
  std::vector<frustum_model> & frustums,
  std::unordered_set<occupany_cell> & cleared_cells)
/*****************************************************************************/
{
  // sample time once for all clearing readings
  const double cur_time = _clock->now().seconds();

  // check each point in the grid for inclusion in a frustum
  openvdb::DoubleGrid::ValueOnCIter cit_grid = _grid->cbeginValueOn();
  for (; cit_grid.test(); ++cit_grid) {
    const openvdb::Coord pt_index(cit_grid.getCoord());
    const openvdb::Vec3d pose_world = this->IndexToWorld(pt_index);

    std::vector<frustum_model>::iterator frustum_it = frustums.begin();
    bool frustum_cycle = false;
    bool cleared_point = false;

    const double time_since_marking = cur_time - cit_grid.getValue();
    const double base_duration_to_decay = GetTemporalClearingDuration(
      time_since_marking);

    for (; frustum_it != frustums.end(); ++frustum_it) {
      if (!frustum_it->frustum) {
        continue;
      }

      if (frustum_it->frustum->IsInside(pose_world) ) {
        frustum_cycle = true;

        const double frustum_acceleration = GetFrustumAcceleration(
          time_since_marking, frustum_it->accel_factor);

        const double time_until_decay = base_duration_to_decay -
          frustum_acceleration;
        if (time_until_decay < 0.) {
          // expired by acceleration
          cleared_point = true;
          if (!this->ClearGridPoint(pt_index)) {
            std::cout << "Failed to clear point." << std::endl;
          }
          break;
        } else {
          const double updated_mark = cit_grid.getValue() -
            frustum_acceleration;
          if (!this->MarkGridPoint(pt_index, updated_mark)) {
            std::cout << "Failed to update mark." << std::endl;
          }
          break;
        }
      }
    }

    // if not inside any, check against nominal decay model
    if (!frustum_cycle) {
      if (base_duration_to_decay < 0.) {
        // expired by temporal clearing
        cleared_point = true;
        if (!this->ClearGridPoint(pt_index)) {
          std::cout << "Failed to clear point." << std::endl;
        }
      }
    }

    if (cleared_point)
    {
      cleared_cells.insert(occupany_cell(pose_world[0], pose_world[1]));
    } else {
      // if here, we can add to costmap and PC2
      PopulateCostmapAndPointcloud(pt_index);
    }
  }

  // free memory taken by expired voxels
  _grid->pruneGrid();
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::PopulateCostmapAndPointcloud(
  const openvdb::Coord & pt)
/*****************************************************************************/
{
  // add pt to the pointcloud and costmap
  openvdb::Vec3d pose_world = this->IndexToWorld(pt);

  if (_pub_voxels) {
    geometry_msgs::msg::Point32 point;
    point.x = pose_world[0];
    point.y = pose_world[1];
    point.z = pose_world[2];
    _grid_points->push_back(point);
  }

  std::unordered_map<occupany_cell, uint>::iterator cell;
  cell = _cost_map->find(occupany_cell(pose_world[0], pose_world[1]));
  if (cell != _cost_map->end()) {
    cell->second += 1;
  } else {
    _cost_map->insert(
      std::make_pair(
        occupany_cell(pose_world[0], pose_world[1]), 1));
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::Mark(
  const std::vector<observation::MeasurementReading> & marking_readings)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  // mark the grid
  if (marking_readings.size() > 0) {
    for (uint i = 0; i != marking_readings.size(); i++) {
      (*this)(marking_readings.at(i));
    }
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::operator()(
  const observation::MeasurementReading & obs) const
/*****************************************************************************/
{
  if (obs._marking) {
    float mark_range_2 = obs._obstacle_range_in_m * obs._obstacle_range_in_m;
    const double cur_time = _clock->now().seconds();

    const sensor_msgs::msg::PointCloud2 & cloud = *(obs._cloud);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end();
      ++iter_x, ++iter_y, ++iter_z)
    {
      float distance_2 =
        (*iter_x - obs._origin.x) * (*iter_x - obs._origin.x) +
        (*iter_y - obs._origin.y) * (*iter_y - obs._origin.y) +
        (*iter_z - obs._origin.z) * (*iter_z - obs._origin.z);
      if (distance_2 > mark_range_2 || distance_2 < 0.0001) {
        continue;
      }

      double x = *iter_x < 0 ? *iter_x - _voxel_size : *iter_x;
      double y = *iter_y < 0 ? *iter_y - _voxel_size : *iter_y;
      double z = *iter_y < 0 ? *iter_z - _voxel_size : *iter_z;

      openvdb::Vec3d mark_grid(this->WorldToIndex(
          openvdb::Vec3d(x, y, z)));

      if (!this->MarkGridPoint(
          openvdb::Coord(
            mark_grid[0], mark_grid[1],
            mark_grid[2]), cur_time))
      {
        std::cout << "Failed to mark point." << std::endl;
      }
    }
  }
}

/*****************************************************************************/
std::unordered_map<occupany_cell, uint> *
SpatioTemporalVoxelGrid::GetFlattenedCostmap()
/*****************************************************************************/
{
  return _cost_map;
}

/*****************************************************************************/
double SpatioTemporalVoxelGrid::GetTemporalClearingDuration(
  const double & time_delta)
/*****************************************************************************/
{
  // use configurable model to get desired decay time
  if (_decay_model == 0) {  // Linear
    return _voxel_decay - time_delta;
  } else if (_decay_model == 1) {  // Exponential
    return _voxel_decay * std::exp(-time_delta);
  }
  return _voxel_decay;  // PERSISTENT
}

/*****************************************************************************/
double SpatioTemporalVoxelGrid::GetFrustumAcceleration(
  const double & time_delta, const double & acceleration_factor)
/*****************************************************************************/
{
  const double acceleration = 1. / 6. * acceleration_factor *
    (time_delta * time_delta * time_delta);
  return acceleration;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::GetOccupancyPointCloud(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> & pc2)
/*****************************************************************************/
{
  // convert the grid points stored in a PointCloud2
  pc2->width = _grid_points->size();
  pc2->height = 1;
  pc2->is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(*pc2);

  modifier.setPointCloud2Fields(
    3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2, "z");

  for (std::vector<geometry_msgs::msg::Point32>::iterator it =
    _grid_points->begin();
    it != _grid_points->end(); ++it)
  {
    const geometry_msgs::msg::Point32 & pt = *it;
    *iter_x = pt.x;
    *iter_y = pt.y;
    *iter_z = pt.z;
    ++iter_x; ++iter_y; ++iter_z;
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::ResetGrid(void)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  // clear the voxel grid
  try {
    _grid->clear();
    if (this->IsGridEmpty()) {
      return true;
    }
  } catch (...) {
    std::cout << "Failed to reset costmap, please try again." << std::endl;
  }
  return false;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::ResetGridArea(
  const occupany_cell & start, const occupany_cell & end, bool invert_area)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  openvdb::DoubleGrid::ValueOnCIter cit_grid = _grid->cbeginValueOn();
  for (cit_grid; cit_grid.test(); ++cit_grid)
  {
    const openvdb::Coord pt_index(cit_grid.getCoord());
    const openvdb::Vec3d pose_world = this->IndexToWorld(pt_index);

    const bool in_x_range = pose_world.x() > start.x && pose_world.x() < end.x;
    const bool in_y_range = pose_world.y() > start.y && pose_world.y() < end.y;
    const bool in_range = in_x_range && in_y_range;

    if(in_range != invert_area)
    {
      ClearGridPoint(pt_index);
    }
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::MarkGridPoint(
  const openvdb::Coord & pt, const double & value) const
/*****************************************************************************/
{
  // marking the OpenVDB set
  openvdb::DoubleGrid::Accessor accessor = _grid->getAccessor();

  accessor.setValueOn(pt, value);
  return accessor.getValue(pt) == value;
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::ClearGridPoint(const openvdb::Coord & pt) const
/*****************************************************************************/
{
  // clearing the OpenVDB set
  openvdb::DoubleGrid::Accessor accessor = _grid->getAccessor();

  if (accessor.isValueOn(pt)) {
    accessor.setValueOff(pt, _background_value);
  }
  return !accessor.isValueOn(pt);
}

/*****************************************************************************/
openvdb::Vec3d SpatioTemporalVoxelGrid::IndexToWorld(
  const openvdb::Coord & coord) const
/*****************************************************************************/
{
  // Applies tranform stored in getTransform.
  openvdb::Vec3d pose_world =  _grid->indexToWorld(coord);

  // Using the center for world coordinate
  const double & center_offset = _voxel_size / 2.0;
  pose_world[0] += center_offset;
  pose_world[1] += center_offset;
  pose_world[2] += center_offset;

  return pose_world;
}

/*****************************************************************************/
openvdb::Vec3d SpatioTemporalVoxelGrid::WorldToIndex(
  const openvdb::Vec3d & vec) const
/*****************************************************************************/
{
  // Applies inverse tranform stored in getTransform.
  return _grid->worldToIndex(vec);
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::IsGridEmpty(void) const
/*****************************************************************************/
{
  // Returns grid's population status
  return _grid->empty();
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::SaveGrid(
  const std::string & file_name, double & map_size_bytes)
/*****************************************************************************/
{
  try {
    openvdb::io::File file(file_name + ".vdb");
    openvdb::GridPtrVec grids = {_grid};
    file.write(grids);
    file.close();
    map_size_bytes = _grid->memUsage();
    return true;
  } catch (...) {
    map_size_bytes = 0.;
    return false;
  }
  return false;  // best offense is a good defense
}

}  // namespace volume_grid
