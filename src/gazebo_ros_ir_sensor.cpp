#include <algorithm>
#include <string>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "pheeno_ros_sim/gazebo_ros_ir_sensor.h"


namespace gazebo {
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosIrSensor)

  // Constructor
  GazeboRosIrSensor::GazeboRosIrSensor() {
    this->seed = 0;
  }

  // Destructor
  GazeboRosIrSensor::~GazeboRosIrSensor() {
    this->rosnode_->shutdown();
    delete this->rosnode_;
  }

  // Load the controller
  void GazeboRosIrSensor::Load(sensors::SensorPtr _parent,
                               sdf::ElementPtr _sdf) {
    // Load plugin
    RayPlugin::Load(_parent, this->sdf);

    // Get the world name.
    # if GAZEBO_MAJOR_VERSION >= 7
      std::string worldName = _parent->WorldName();
    # else
      std::string worldName = _parent->GetWorldName();
    #endif

    this->world_ = physics::get_world(worldName);

    // Save pointers
    this->sdf = _sdf;

    GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
    this->parent_ray_sensor_ =
      dynamic_pointer_cast<sensors::RaySensor>(_parent);

    if (!this->parent_ray_sensor_)
      gzthrow("GazeboRosIrSensor controller requires a Ray Sensor as its parent");

    this->robot_namespace_ = GetRobotNamespace(_parent, _sdf, "IR_Sensor");

    if (!this->sdf->HasElement("frameName")) {
      ROS_INFO_NAMED(
        "ir_sensor",
        "IR Sensor plugin missing <frameName>, defaults to world");
      this->frame_name_ = "/world";

    } else {
      this->frame_name_ = this->sdf->Get<std::string>("frameName");

    }

    if (!this->sdf->HasElement("topicName")) {
      ROS_INFO_NAMED(
        "ir_sensor",
        "IR Sensor plugin missing <topicName>, defaults to /world");
      this->topic_name_ = "/world";

    } else {
      this->topic_name_ = this->sdf->Get<std::string>("topicName");

    }

    this->ir_sensor_connect_count_ = 0;

    // Make sure the ROS node for Gazebo has already been initialized.
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM_NAMED("ir_sensor", "A ROS node for Gazebo has not been initialized, unable to load plugin. " << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    ROS_INFO_NAMED(
      "ir_sensor",
      "Starting IR Sensor Plugin (ns = %s)",
      this->robot_namespace_.c_str());

    // ros callback queue for processing subscription
    this->deferred_load_thread_ = boost::thread(
      boost::bind(&GazeboRosIrSensor::LoadThread, this));
  }

  // Load the controller
  void GazeboRosIrSensor::LoadThread() {
    this->gazebo_node_ = gazebo::transport::NodePtr(
      new gazebo::transport::Node());
    this->gazebo_node_->Init(this->world_name_);

    this->pmq.startServiceThread();

    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    this->tf_prefix_ = tf::getPrefixParam(*this->rosnode_);
    if (this->tf_prefix_.empty()) {
      this->tf_prefix_ = this->robot_namespace_;
      boost::trim_right_if(this->tf_prefix_, boost::is_any_of("/"));

    }

    ROS_INFO_NAMED(
      "ir_sensor",
      "IR Sensor Plugin (ns = %s) <tf_prefix>, set to \"%s\"",
      this->robot_namespace_.c_str(),
      this->tf_prefix_.c_str());

    // Resolve tf prefix
    this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);

    if (this->topic_name_ != "") {
      ros::AdvertiseOptions ao =
        ros::AdvertiseOptions::create<std_msgs::Float32>(
          this->topic_name_,
          1,
          boost::bind(&GazeboRosIrSensor::IrSensorConnect, this),
          boost::bind(&GazeboRosIrSensor::IrSensorDisconnect, this),
          ros::VoidPtr(),
          NULL);
      this->pub_ = this->rosnode_->advertise(ao);
      this->pub_queue_ = this->pmq.addPub<std_msgs::Float32>();
    }

    // Initialize the controller

    // Sensor generation off by default
    this->parent_ray_sensor_->SetActive(false);

  }

  // Increment Count
  void GazeboRosIrSensor::IrSensorConnect() {
    this->ir_sensor_connect_count_++;
    if (this->ir_sensor_connect_count_ == 1) {
      # if GAZEBO_MAJOR_VERSION >= 7
        this->ir_sensor_scan_sub_ =
          this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                        &GazeboRosIrSensor::OnScan, this);
      # else
        this->ir_sensor_scan_sub_ =
          this->gazebo_node_->Subscribe(this->parent_ray_sensor_->GetTopic(),
                                        &GazeboRosIrSensor::OnScan, this);
      # endif
    }
  }

  // Decrement Count
  void GazeboRosIrSensor::IrSensorDisconnect() {
    this->ir_sensor_connect_count_--;
    if (this->ir_sensor_connect_count_ == 0) {
      this->ir_sensor_scan_sub_.reset();
    }
  }

  /**************************************************************
   * Convert New Gazebo Message to ROS message and publish it
   *
   * From my research with the values returned in scan(). The maximum and
   * minimum limits for the ray specified in the robot's XML file (look at
   * the ir_sensor.urdf.xacro file within this repository for reference) are
   * the values returned by `_msg->scan().ranges()`.
   *
   * 720 values are returned by `_msg->scan().ranges()`, however, they all seem
   * to be the same information. Either .3 if nothing is in front of the sensor
   * or lesser values pertaining to the distance from the sensor. If a box is
   * in front of the sensor 0.10 m from the pheeno, ALL the values of
   * `_msg->scan().ranges()` will read about 0.10 m.
   *
   * Since my inquiring lead to this conclusion, I will only use the first
   * value coming from `_msg->scan().ranges(0)` and then multiplying it by
   * 100 to return the values in cm instead of m.
   **************************************************************/
  void GazeboRosIrSensor::OnScan(ConstLaserScanStampedPtr &_msg) {
    // Declare ROS Message
    std_msgs::Float32 ir_sensor_msg;

    // Set the sensor value.
    ir_sensor_msg.data = (float)_msg->scan().ranges(0) * 100.0;

    // Publish message.
    this->pub_queue_->push(ir_sensor_msg, this->pub_);
  }
}
