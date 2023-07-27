/*
  MIT License

  Copyright (c) 2021 Hongkai Ye (kyle_yeh@163.com, hkye@zju.edu.cn)

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
#include <nav_msgs/Odometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

using std::vector;

ros::Publisher _snesor_range_publisher;
ros::Publisher _pub_cloud;
ros::Subscriber _odom_sub;
ros::Subscriber _global_map_sub;
ros::Timer _local_sensing_timer;

int car_id;
bool _has_global_map(false);
bool _has_odom(false);
bool _use_resolution_filter(true);
bool _hrz_limited(false);

double _sensing_horizon, _sensing_rate, _pc_resolution;
int _hrz_laser_line_num, _vtc_laser_line_num;

double _hrz_resolution_rad, _vtc_resolution_rad, _vtc_laser_range_rad;
double _half_vtc_resolution_and_half_range;
double _half_hrz_range(M_PI);
nav_msgs::Odometry _odom;
Eigen::MatrixXi _idx_map;
Eigen::MatrixXd _dis_map;

pcl::PointCloud<pcl::PointXYZ> _cloud_all_map, _local_map;
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
sensor_msgs::PointCloud2 _local_map_pcd;

pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
vector<int> _pointIdxRadiusSearch;
vector<float> _pointRadiusSquaredDistance;

/**
 * @brief
 * Given a line and a plane, calculating for the intersection and the distance.
 * line_dir is required to be a normalized vector.
 * @param intersection the intersecting point.
 * @param line_p A point in the line.
 * @param line_dir The line direction vector.
 * @param plane_p A point in the plane.
 * @param plane_normal The plane normal vector.
 * @return double
 * The distance between the query point and the intersection.
 * A negtive value means the line direction vector points away from the plane.
 */
inline double lineIntersectPlane(Eigen::Vector3d &intersection,
                                 const Eigen::Vector3d &line_p, const Eigen::Vector3d &line_dir,
                                 const Eigen::Vector3d &plane_p, const Eigen::Vector3d &plane_normal)
{
  double d = (plane_p - line_p).dot(plane_normal) / line_dir.dot(plane_normal);
  intersection = line_p + d * line_dir;
  return d;
}

/**
 * @brief
 * filter the points not in range
 * @param idx
 * @param pt
 * @param laser_t
 * @param laser_R must be normalized in each column
 * @return true
 * @return false
 */
inline bool pt2LaserIdx(Eigen::Vector2i &idx,
                        const Eigen::Vector3d &pt,
                        const Eigen::Vector3d &laser_t, const Eigen::Matrix3d &laser_R)
{
  Eigen::Vector3d inter_p;
  double dis_pt_to_laser_plane = lineIntersectPlane(inter_p, pt, laser_R.col(2), laser_t, laser_R.col(2));
  double dis_laser_to_inter_p = (inter_p - laser_t).norm();
  double vtc_rad = std::atan2((pt - laser_t).dot(laser_R.col(2)), dis_laser_to_inter_p);
  if (std::fabs(vtc_rad) >= _half_vtc_resolution_and_half_range)
    return false;

  double x_in_roll_pitch_plane = (inter_p - laser_t).dot(laser_R.col(0));
  double y_in_roll_pitch_plane = (inter_p - laser_t).dot(laser_R.col(1));
  double hrz_rad = std::atan2(y_in_roll_pitch_plane, x_in_roll_pitch_plane);
  if (_hrz_limited && std::fabs(hrz_rad) >= _half_hrz_range)
    return false;

  vtc_rad += _half_vtc_resolution_and_half_range;
  int vtc_idx = std::floor(vtc_rad / _vtc_resolution_rad);
  if (vtc_idx >= _vtc_laser_line_num)
    vtc_idx = 0;

  hrz_rad += M_PI + _hrz_resolution_rad / 2.0;
  int hrz_idx = std::floor(hrz_rad / _hrz_resolution_rad);
  if (hrz_idx >= _hrz_laser_line_num)
    hrz_idx = 0;

  idx << hrz_idx, vtc_idx;
  return true;
}

/**
 * @brief
 *
 * @param x
 * @param y
 * @param dis
 * @param pt in laser coordinate.
 */
inline void idx2Pt(int x, int y, double dis, Eigen::Vector3d &pt)
{
  double vtc_rad = y * _vtc_resolution_rad - _vtc_laser_range_rad / 2.0;
  double hrz_rad = x * _hrz_resolution_rad - M_PI;
  pt[2] = sin(vtc_rad) * dis; // z_in_laser_coor
  double xy_square_in_laser_coor = cos(vtc_rad) * dis;
  pt[0] = cos(hrz_rad) * xy_square_in_laser_coor; // x_in_laser_coor
  pt[1] = sin(hrz_rad) * xy_square_in_laser_coor; // y_in_laser_coor
}

void addSlice(visualization_msgs::Marker &line_msg, double angle)
{
  geometry_msgs::Point p_msg;
  p_msg.x = 0.0;
  p_msg.y = 0.0;
  p_msg.z = 0.0;
  line_msg.points.push_back(p_msg);
  p_msg.x = _sensing_horizon * cos(angle);
  p_msg.y = _sensing_horizon * sin(angle);
  p_msg.z = _sensing_horizon * sin(_half_vtc_resolution_and_half_range);
  line_msg.points.push_back(p_msg);

  p_msg.x = 0.0;
  p_msg.y = 0.0;
  p_msg.z = 0.0;
  line_msg.points.push_back(p_msg);
  p_msg.x = _sensing_horizon * cos(angle);
  p_msg.y = _sensing_horizon * sin(angle);
  p_msg.z = -_sensing_horizon * sin(_half_vtc_resolution_and_half_range);
  line_msg.points.push_back(p_msg);

  p_msg.x = _sensing_horizon * cos(angle);
  p_msg.y = _sensing_horizon * sin(angle);
  p_msg.z = _sensing_horizon * sin(_half_vtc_resolution_and_half_range);
  line_msg.points.push_back(p_msg);
  p_msg.x = _sensing_horizon * cos(angle);
  p_msg.y = _sensing_horizon * sin(angle);
  p_msg.z = -_sensing_horizon * sin(_half_vtc_resolution_and_half_range);
  line_msg.points.push_back(p_msg);
}

void visRange()
{
  // visualize sensor range
  visualization_msgs::Marker clear_previous_msg;
  clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;
  visualization_msgs::Marker arc_msg;
  arc_msg.type = visualization_msgs::Marker::LINE_STRIP;
  arc_msg.action = visualization_msgs::Marker::ADD;
  arc_msg.header.frame_id = "laser"+std::to_string(car_id);
  arc_msg.id = 0;
  arc_msg.pose.position.x = 0;
  arc_msg.pose.position.y = 0;
  arc_msg.pose.position.z = 0;
  arc_msg.pose.orientation.w = 1;
  arc_msg.pose.orientation.x = 0;
  arc_msg.pose.orientation.y = 0;
  arc_msg.pose.orientation.z = 0;
  arc_msg.scale.x = 0.1;
  arc_msg.scale.y = 0.1;
  arc_msg.scale.z = 0.1;
  arc_msg.color.a = 1.0;
  arc_msg.color.r = 0.0;
  arc_msg.color.g = 0.0;
  arc_msg.color.b = 0.0;
  visualization_msgs::MarkerArray path_list_msg;
  path_list_msg.markers.push_back(clear_previous_msg);
  geometry_msgs::Point p_msg;

  /* up and down circle*/
  visualization_msgs::Marker arc_msg_down(arc_msg);
  arc_msg_down.id += 1;
  int num = ceil(_half_hrz_range / 0.05);
  double step = 2 * _half_hrz_range / (double)num;
  for (int i = 0; i <= num; i++)
  {
    double theta = step * i - _half_hrz_range;
    p_msg.x = _sensing_horizon * cos(theta);
    p_msg.y = _sensing_horizon * sin(theta);
    p_msg.z = _sensing_horizon * sin(_half_vtc_resolution_and_half_range);
    arc_msg.points.push_back(p_msg);
    p_msg.z *= -1.0;
    arc_msg_down.points.push_back(p_msg);
  }
  path_list_msg.markers.push_back(arc_msg);
  path_list_msg.markers.push_back(arc_msg_down);

  /* lines*/
  visualization_msgs::Marker line_msg(arc_msg_down);
  line_msg.id += 1;
  line_msg.points.clear();
  line_msg.type = visualization_msgs::Marker::LINE_LIST;

  addSlice(line_msg, _half_hrz_range);
  addSlice(line_msg, -_half_hrz_range);

  path_list_msg.markers.push_back(line_msg);
  _snesor_range_publisher.publish(path_list_msg);
}

/**
 * @brief
 *
 * @param laser_odom laser odometry
 */
void rcvOdometryCallbck(const nav_msgs::Odometry::ConstPtr &laser_odom)
{
  _has_odom = true;
  _odom.pose = laser_odom->pose;
  _odom.header = laser_odom->header;

  // maintain tf: world->laser
  static tf2_ros::TransformBroadcaster _br_world_laser;
  geometry_msgs::TransformStamped _transformStamped;
  _transformStamped.header.stamp = laser_odom->header.stamp;
  if (laser_odom->header.frame_id.size())
    _transformStamped.header.frame_id = laser_odom->header.frame_id;
  else
    _transformStamped.header.frame_id = "world";
  _transformStamped.child_frame_id = "laser"+std::to_string(car_id);
  _transformStamped.transform.translation.x = laser_odom->pose.pose.position.x;
  _transformStamped.transform.translation.y = laser_odom->pose.pose.position.y;
  _transformStamped.transform.translation.z = laser_odom->pose.pose.position.z;
  _transformStamped.transform.rotation.x = laser_odom->pose.pose.orientation.x;
  _transformStamped.transform.rotation.y = laser_odom->pose.pose.orientation.y;
  _transformStamped.transform.rotation.z = laser_odom->pose.pose.orientation.z;
  _transformStamped.transform.rotation.w = laser_odom->pose.pose.orientation.w;
  _br_world_laser.sendTransform(_transformStamped);

  // visualize sensor range
  visRange();
}

void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map)
{
  if (_has_global_map)
    return;

  ROS_WARN("Global Pointcloud received..");

  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(pointcloud_map, cloud_input);

  _voxel_sampler.setLeafSize(_pc_resolution, _pc_resolution, _pc_resolution);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  _voxel_sampler.filter(_cloud_all_map);

  _kdtreeLocalMap.setInputCloud(_cloud_all_map.makeShared());

  _has_global_map = true;
  _global_map_sub.shutdown();
}

void renderSensedPoints(const ros::TimerEvent &event)
{
  double this_time = ros::Time::now().toSec();
  if (!_has_global_map)
  {
    ROS_ERROR("[Laser sim] No global map received yet.");
    return;
  }
  if (!_has_odom)
  {
    ROS_ERROR("[Laser sim] No odometry information.");
    return;
  }

  Eigen::Quaterniond q;
  q.x() = _odom.pose.pose.orientation.x;
  q.y() = _odom.pose.pose.orientation.y;
  q.z() = _odom.pose.pose.orientation.z;
  q.w() = _odom.pose.pose.orientation.w;
  Eigen::Matrix3d rot(q);
  Eigen::Vector3d laser_t(_odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z);

  _pointIdxRadiusSearch.clear();
  _pointRadiusSquaredDistance.clear();
  pcl::PointXYZ searchPoint(_odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z);
  _kdtreeLocalMap.radiusSearch(searchPoint, _sensing_horizon, _pointIdxRadiusSearch, _pointRadiusSquaredDistance);

  _idx_map.setConstant(-1);
  _dis_map.setConstant(9999.0);

  pcl::PointXYZ pt;
  for (size_t i = 0; i < _pointIdxRadiusSearch.size(); ++i)
  {
    pt = _cloud_all_map.points[_pointIdxRadiusSearch[i]];

    Eigen::Vector2i idx;
    bool in_range = pt2LaserIdx(idx, Eigen::Vector3d(pt.x, pt.y, pt.z), laser_t, rot);
    if (!in_range)
      continue;
    Eigen::Vector3d pt_vec(pt.x - _odom.pose.pose.position.x, pt.y - _odom.pose.pose.position.y, pt.z - _odom.pose.pose.position.z);
    double dis_curr_pt = pt_vec.norm();

    if (_use_resolution_filter)
    {
      double vtc_rad = idx[1] * _vtc_resolution_rad - _vtc_laser_range_rad / 2.0;
      double dis_to_z_axis = dis_curr_pt * cos(vtc_rad);
      double mesh_len_hrz = dis_to_z_axis * _hrz_resolution_rad;
      double mesh_len_vtc = dis_to_z_axis * _vtc_resolution_rad;
      int hrz_occ_grid_num = std::min((int)floor(_pc_resolution / mesh_len_hrz), _hrz_laser_line_num);
      int vtc_occ_grid_num = std::min((int)floor(_pc_resolution / mesh_len_vtc), _vtc_laser_line_num);
      // ROS_INFO_STREAM("hrz_occ_grid_num " << hrz_occ_grid_num / 2 << ", vtc_occ_grid_num " << vtc_occ_grid_num / 2);
      int tmp1 = hrz_occ_grid_num, tmp2 = vtc_occ_grid_num;
      for (int d_hrz_idx = -tmp1; d_hrz_idx <= tmp1; ++d_hrz_idx)
        for (int d_vtc_idx = -tmp2; d_vtc_idx <= tmp2; ++d_vtc_idx)
        {
          int hrz_idx = (idx[0] + d_hrz_idx + _hrz_laser_line_num) % _hrz_laser_line_num; // it's a ring in hrz coordiante
          int vtc_idx = idx[1] + d_vtc_idx;
          if (vtc_idx >= _vtc_laser_line_num)
            continue;
          if (vtc_idx < 0)
            continue;
          // ROS_INFO_STREAM("hrz_idx " << hrz_idx << ", vtc_idx " << vtc_idx);
          if (dis_curr_pt < _dis_map(hrz_idx, vtc_idx))
          {
            _idx_map(hrz_idx, vtc_idx) = i;
            _dis_map(hrz_idx, vtc_idx) = dis_curr_pt;
          }
        }
    }
    else
    {
      if (dis_curr_pt < _dis_map(idx[0], idx[1]))
      {
        _idx_map(idx[0], idx[1]) = i;
        _dis_map(idx[0], idx[1]) = dis_curr_pt;
      }
    }
  }

  // ROS_INFO("render cost %lf ms.", (ros::Time::now().toSec() - this_time) * 1000.0f);
  _local_map.points.clear();
  for (int x = 0; x < _hrz_laser_line_num; ++x)
    for (int y = 0; y < _vtc_laser_line_num; ++y)
    {
      /* use map cloud pts as laser pts
      if (idx_map(x, y) == -1)
        continue;
      pt = _cloud_all_map.points[_pointIdxRadiusSearch[idx_map(x, y)]];
      _local_map.points.push_back(pt);
      */

      // use laser line pts
      Eigen::Vector3d p;
      if (_idx_map(x, y) != -1)
      {
        idx2Pt(x, y, _dis_map(x, y), p);
        _local_map.points.emplace_back(p[0], p[1], p[2]);
      }
    }

  _local_map.width = _local_map.points.size();
  _local_map.height = 1;
  _local_map.is_dense = true;

  pcl::toROSMsg(_local_map, _local_map_pcd);
  _local_map_pcd.header.frame_id = "laser"+std::to_string(car_id);
  _local_map_pcd.header.stamp = _odom.header.stamp;

  _pub_cloud.publish(_local_map_pcd);
  // ROS_INFO("all cost %lf ms.", (ros::Time::now().toSec() - this_time) * 1000.0f);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_simulator");
  ros::NodeHandle nh("~");

  nh.getParam("car_id", car_id);
  nh.getParam("sensing_horizon", _sensing_horizon);
  nh.getParam("sensing_rate", _sensing_rate);
  nh.getParam("pc_resolution", _pc_resolution);
  nh.getParam("use_resolution_filter", _use_resolution_filter);
  nh.getParam("hrz_limited", _hrz_limited);

  nh.getParam("hrz_laser_line_num", _hrz_laser_line_num);
  nh.getParam("vtc_laser_line_num", _vtc_laser_line_num);

  double vtc_laser_range_dgr;
  nh.getParam("vtc_laser_range_dgr", vtc_laser_range_dgr);
  _vtc_laser_range_rad = vtc_laser_range_dgr / 180.0 * M_PI;
  _vtc_resolution_rad = _vtc_laser_range_rad / (double)(_vtc_laser_line_num - 1);
  _half_vtc_resolution_and_half_range = (_vtc_laser_range_rad + _vtc_resolution_rad) / 2.0;

  double hrz_laser_range_dgr;
  nh.getParam("hrz_laser_range_dgr", hrz_laser_range_dgr);
  if (_hrz_limited)
  {
    _half_hrz_range = hrz_laser_range_dgr / 180.0 * M_PI / 2.0;
  }
  _hrz_resolution_rad = 2 * M_PI / (double)_hrz_laser_line_num;

  _idx_map = Eigen::MatrixXi::Constant(_hrz_laser_line_num, _vtc_laser_line_num, -1);
  _dis_map = Eigen::MatrixXd::Constant(_hrz_laser_line_num, _vtc_laser_line_num, 9999.0);

  std::string map_topic, odom_topic, laser_pcd_topic, laser_range_topic;
  nh.getParam("map_topic", map_topic);
  nh.getParam("odom_topic", odom_topic);
  nh.getParam("laser_pcd_topic", laser_pcd_topic);
  nh.getParam("laser_range_topic", laser_range_topic);
  _global_map_sub = nh.subscribe(map_topic, 1, rcvGlobalPointCloudCallBack);
  _odom_sub = nh.subscribe(odom_topic, 50, rcvOdometryCallbck);
  _pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(laser_pcd_topic, 10);
  _snesor_range_publisher = nh.advertise<visualization_msgs::MarkerArray>(laser_range_topic, 1);
  double sensing_duration = 1.0 / _sensing_rate;
  _local_sensing_timer = nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);

  ros::spin();
}
