#include "mrs_lib/safety_zone/static_edges_visualization.h"
#include "mrs_lib/attitude_converter.h"
#include <geometry_msgs/Point.h>
#include <atomic>

namespace vm = visualization_msgs;
namespace gm = geometry_msgs;

namespace mrs_lib
{

  std::atomic<int> StaticEdgesVisualization::id_generator_ = 0;
  visualization_msgs::MarkerArray StaticEdgesVisualization::last_markers_ = visualization_msgs::MarkerArray();
  std::mutex StaticEdgesVisualization::last_markers_mutex_;
  std::unordered_set<const StaticEdgesVisualization*> StaticEdgesVisualization::live_instances_;

  /* StaticEdgesVisualization(prism) //{ */

  StaticEdgesVisualization::StaticEdgesVisualization(Prism* prism, const std::string& uav_name, const std::string& frame_id, const ros::NodeHandle& nh,
                                                     const double& markers_update_rate)
      : id_(id_generator_++), prism_(prism), uav_name_(uav_name), frame_id_(frame_id), nh_(nh)
  {
    init(markers_update_rate);
  }

  //}

  /* StaticEdgesVisualization(obstacle) //{ */

  StaticEdgesVisualization::StaticEdgesVisualization(SafetyZone* safety_zone, const int& obstacle_id, const std::string& uav_name, const std::string& frame_id,
                                                     const ros::NodeHandle& nh, const double& markers_update_rate)

      : id_(id_generator_++), prism_(safety_zone->getObstacle(obstacle_id)), uav_name_(uav_name), frame_id_(frame_id), nh_(nh)
  {
    init(markers_update_rate);
  }

  //}

  /* StaticEdgesVisualization(safety border) //{ */

  StaticEdgesVisualization::StaticEdgesVisualization(SafetyZone* safety_zone, const std::string& uav_name, const std::string& frame_id,
                                                     const ros::NodeHandle& nh, const double& markers_update_rate)

      : id_(id_generator_++), prism_(safety_zone->getBorder()), uav_name_(uav_name), frame_id_(frame_id), nh_(nh)
  {
    init(markers_update_rate);
  }

  //}

  /* ~StaticEdgesVisualization() //{ */

  StaticEdgesVisualization::~StaticEdgesVisualization()
  {
    {
      std::scoped_lock lock(last_markers_mutex_);
      live_instances_.erase(this);
    }
    timer_.stop();
    if (is_active_)
    {
      prism_->unsubscribe(this);
    }
    cleanup();
  }

  //}

  /* init() //{ */

  void StaticEdgesVisualization::init(const double markers_update_rate)
  {
    {
      // size by id: instances can be created out of order
      std::scoped_lock lock(last_markers_mutex_);
      if (last_markers_.markers.size() <= static_cast<size_t>(id_))
      {
        last_markers_.markers.resize(id_ + 1);
      }
      live_instances_.insert(this);
    }

    publisher_ = nh_.advertise<vm::MarkerArray>("safety_area_static_markers_out", 1);
    prism_->subscribe(this);
    update();
    is_active_ = true;  // is_active_ flag inherited from Subscriber class (defined in Prism)

    // start the timer last, it fires on another thread
    timer_ = nh_.createTimer(ros::Rate(markers_update_rate), &StaticEdgesVisualization::sendMarker, this);
  }

  //}

  /* sendMarker() //{ */

  // Lines must be updated periodically. View https://github.com/ros-visualization/rviz/issues/1287
  void StaticEdgesVisualization::sendMarker([[maybe_unused]] const ros::TimerEvent& event)
  {
    std::scoped_lock lock(last_markers_mutex_);
    if (live_instances_.count(this) == 0)
    {
      return;  // already destroyed
    }
    publisher_.publish(last_markers_);
  }

  //}

  /* update() //{ */

  void StaticEdgesVisualization::update()
  {
    if (!is_active_)
    {
      return;
    }

    const double max_z = prism_->getMaxZ();
    const double min_z = prism_->getMinZ();
    const auto polygon = prism_->getPolygon().outer();

    vm::Marker marker;
    marker.id = id_;
    std::string target_frame_id = "world_origin";
    marker.header.frame_id = uav_name_ + "/" + target_frame_id;
    marker.type = vm::Marker::LINE_LIST;
    marker.action = vm::Marker::ADD;
    marker.color.a = 0.3;
    marker.color.r = 1.0;
    marker.color.b = 0.0;
    marker.color.g = 0.0;
    marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);
    marker.scale.x = 0.2;

    // Iterating over polygon edges to publish the marker lines

    for (size_t i = 0; i < polygon.size() - 1; i++)
    {  // -1 because the last is the same as the first one
      // Adding upper horizontal edges
      gm::Point upper_start;
      gm::Point upper_end;

      upper_start.x = polygon[i].get<0>();
      upper_start.y = polygon[i].get<1>();
      upper_start.z = max_z;

      upper_end.x = polygon[i + 1].get<0>();
      upper_end.y = polygon[i + 1].get<1>();
      upper_end.z = max_z;

      marker.points.push_back(upper_start);

      marker.points.push_back(upper_end);

      // Adding lower horizontal edges
      gm::Point lower_start;
      gm::Point lower_end;

      lower_start.x = polygon[i].get<0>();
      lower_start.y = polygon[i].get<1>();
      lower_start.z = min_z;

      lower_end.x = polygon[i + 1].get<0>();
      lower_end.y = polygon[i + 1].get<1>();
      lower_end.z = min_z;

      marker.points.push_back(lower_start);

      marker.points.push_back(lower_end);

      // Adding vertical edges
      marker.points.push_back(upper_start);
      marker.points.push_back(lower_start);
    }

    std::scoped_lock lock(last_markers_mutex_);
    last_markers_.markers[id_] = marker;
    publisher_.publish(last_markers_);
  }

  //}

  /* cleanup() //{ */

  void StaticEdgesVisualization::cleanup()
  {
    {
      std::scoped_lock lock(last_markers_mutex_);
      last_markers_.markers[id_].action = vm::Marker::DELETE;
    }
    is_active_ = false;
  }

  //}

}  // namespace mrs_lib
