#ifndef GAZEBO_ROS_IR_SENSOR_HH
#define GAZEBO_ROS_IR_SENSOR_HH

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include "std_msgs/Float32.h"

#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo {
  class GazeboRosIrSensor : public RayPlugin {
    // \brief Constructor
    public: GazeboRosIrSensor();

    // \brief Destructor
    public: ~GazeboRosIrSensor();

    // \brief Load the plugin
    // \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    // \brief Keep track of number of connections
    private: int ir_sensor_connect_count_;
    private: void IrSensorConnect();
    private: void IrSensorDisconnect();

    // Pointer to the model
    GazeboRosPtr gazebo_ros_;
    private: std::string world_name_;
    private: physics::WorldPtr world_;

    // \brief The parent sensor
    private: sensors::RaySensorPtr parent_ray_sensor_;

    // \brief pointer to ros node
    private: ros::NodeHandle *rosnode_;
    private: ros::Publisher pub_;
    private: PubQueue<std_msgs::Float32>::Ptr pub_queue_;

    // \brief topic name
    private: std::string topic_name_;

    // \brief frame transform name, should match link name
    private: std::string frame_name_;

    // \brief tf prefix
    private: std::string tf_prefix_;

    // \brief for setting ROS name space
    private: std::string robot_namespace_;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf_;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;
    private: unsigned int seed_;

    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr ir_sensor_scan_sub_;
    private: void OnScan(ConstLaserScanStampedPtr &_msg);

    // \brief prevent blocking
    private: PubMultiQueue pmq_;
  };
}
#endif
