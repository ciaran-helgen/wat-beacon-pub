#ifndef _BEACON_REPUB_PLUGIN_HH_
#define _BEACON_REPUB_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <boost/shared_ptr.hpp>
#include <string>

#include <ros/ros.h>
#include <beacon_pub/beacon.h>

namespace gazebo
{
class BeaconRepub : public SensorPlugin
{

  // Pointer to sensor instance
  private: sensors::WirelessReceiverPtr receiverSensor;

  // Pointer to SDF element
  private: sdf::ElementPtr sdf;

  //A node used for transport
  private: transport::NodePtr node;

  // Subscriber to gazebo topic (WirelessNodes message)
  private: transport::SubscriberPtr sub;

  // Subscriber to get sim time
  private: transport::SubscriberPtr gz_clock_sub;

  // Incrementing sequence number for ROS msg header
  private: unsigned int sequence_ctr = 0;

  //variables to store simulation time
  private: unsigned int gz_sec;
  private: unsigned int gz_nsec;

  //ROS topic to publish to
  private: std::string topic_name;
  //ROS Frame for beacon msg header
  private: std::string frame_name;

  //ROS node for publisher
  ros::NodeHandle n;

  //ROS publisher
  ros::Publisher pub;


  public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    // Convert the base sensor pointer to a WirelessReceiver pointer
    this->receiverSensor = std::dynamic_pointer_cast<sensors::WirelessReceiver>(_sensor);

    this->sdf = _sdf;

    // Create the node
    this->node = transport::NodePtr(new transport::Node());
    
    this->node->Init();
   
    // Get the frame of the message from the frame_name tag in the plugin, if it exists
    if (!this->sdf->HasElement("frame_name"))
    {
      ROS_INFO("BeaconRepub plugin missing <frameName>, defaults to /world");
      this->frame_name = "/world";
    }
    else
    {
      this->frame_name = this->sdf->Get<std::string>("frame_name");
    }
  
    // Get the topic of the message from the topic_name tag in the plugin, if it exists
    if (!this->sdf->HasElement("topic_name"))
    {
      ROS_INFO("BeaconRepub plugin missing <topicName>, defaults to /receiver");
      this->topic_name = "/receiver";
    }
    else
    {
      this->topic_name = this->sdf->Get<std::string>("topic_name");
    }

    // Gets the topic of the sensor programmatically using the Topic() method
    // Subscribe to the topic, and register a callback
    this->sub = this->node->Subscribe(this->receiverSensor->Topic(), 
                                        &BeaconRepub::BeaconMsgCB, this);

    // subscribe to /world_stats to access sim_time
    this->gz_clock_sub = this->node->Subscribe("~/world_stats", 
                                                  &BeaconRepub::ClockCB, this);

    //Begin publisher for ROS message
    this->pub = n.advertise<beacon_pub::beacon>(this->topic_name, 1000);

  }

  // The callback for the gazebo WirelessNodes message
  public: void BeaconMsgCB(ConstWirelessNodesPtr &gmsg)
  {
    
    if(gmsg!=0)
    {
      // get signal level of zeroth wirelessnode in message 
      // wireless_nodes.proto: message consists of repeated 'node' messages
      // node(n) accesses the nth WirelessNode, whose data can be accessed
      // as normal (essid, signal_level, frequency)
      double gz_signal_level = gmsg->node(0).signal_level();
      double gz_frequency = gmsg->node(0).frequency();
      std::string gz_essid = gmsg->node(0).essid();

      beacon_pub::beacon rosmsg;
      // Build the ROS message
      // Header
      rosmsg.header.seq = this->sequence_ctr;
      rosmsg.header.stamp.sec = this->gz_sec;
      rosmsg.header.stamp.nsec = this->gz_nsec;
      rosmsg.header.frame_id = this->frame_name;

      // beacon params
      rosmsg.signal_level = gz_signal_level;
      rosmsg.essid = gz_essid;
      rosmsg.frequency = gz_frequency;

      this->pub.publish(rosmsg);
      ros::spinOnce();
      this->sequence_ctr ++;
      //ROS_INFO("Callback Called! WirelessNodes Message Received");
    }
    //ROS_INFO("Callback Called! No WirelessNodes msg");
 
  }

  // The callback for the WorldStatistics message (sim_time)
  public: void ClockCB(ConstWorldStatisticsPtr &gmsg)
  {
    if(gmsg!=0)
    {
      this->gz_sec = gmsg->sim_time().sec();
      this->gz_nsec = gmsg->sim_time().nsec();
    }
  }

};

GZ_REGISTER_SENSOR_PLUGIN(BeaconRepub)
}
#endif