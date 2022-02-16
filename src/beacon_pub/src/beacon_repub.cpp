#ifndef _BEACON_REPUB_PLUGIN_HH_
#define _BEACON_REPUB_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <boost/algorithm/string/replace.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <beacon_pub/beacon.h>

namespace gazebo
{
class BeaconRepub : public ModelPlugin
{

  private: physics::ModelPtr model;

  //A node used for transport
  private: transport::NodePtr node;

  // A subscriber to a named topic.
  private: transport::SubscriberPtr sub;

  private: unsigned int sequence_ctr = 0;

  private: std::string frame_id;

  //ROS node for publisher
  ros::NodeHandle n;

  //ROS publisher
  ros::Publisher pub;


  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->model = _parent;

    // Create the node
    this->node = transport::NodePtr(new transport::Node());
    
    this->node->Init();
    std::string nodename = this->model->GetWorld()->Name();
    ROS_INFO(nodename.c_str());
    // TODO: Get topic name using sdf or 'Name()/GetName()/GetLink()->Name() etc. 
    // Create a topic name
    std::string topicName = "/gazebo/default/wirelessReceiver/link/wirelessReceiver/transceiver";

    // Subscribe to the topic, and register a callback
    this->sub = this->node->Subscribe(topicName, &BeaconRepub::OnMsg, this);

    //Begin publisher for ROS message
    //commented out example float32 publisher
    //this->pub = n.advertise<std_msgs::Float32>("receiver", 1000);
    this->pub = n.advertise<beacon_pub::beacon>("receiver", 1000);

  }

  public: void OnMsg(ConstWirelessNodesPtr &gmsg)
  {
    
    //if(gmsg!=0)
    //{
      // get signal level of zeroth wirelessnode in message 
      // wireless_nodes.proto: message consists of repeated 'node' messages
      // node(n) accesses the nth WirelessNode, whose data can be accessed
      // as normal (essid, signal_level, frequency)
      double gz_signal_level = gmsg->node(0).signal_level();
      double gz_frequency = gmsg->node(0).frequency();
      std::string gz_essid = gmsg->node(0).essid();
      
      

      //std::cout << "Signal Level: " << siglevel << std::endl;

      //commented out example float32 message
      //std_msgs::Float32 rosmsg;
      beacon_pub::beacon rosmsg;
      // //build message
      //Header
      rosmsg.header.seq = this->sequence_ctr;
      //TODO: Get sim time from /clock
        //TODO: Only get if /use_sim_time param is true
      rosmsg.header.stamp = ros::Time::now();
      //TODO: get frame ID from sdf. Store frame ID as parameter of this class
      rosmsg.header.frame_id = "placeholder_frame"; //this->frame_id

      // beacon info
      rosmsg.signal_level = gz_signal_level;
      rosmsg.essid = gz_essid;
      rosmsg.frequency = gz_frequency;

      this->pub.publish(rosmsg);
      ros::spinOnce();
      this->sequence_ctr ++;
    //}
    ROS_INFO("Callback Called!");
     // Create ROS node and init
    
    
  }

};

GZ_REGISTER_MODEL_PLUGIN(BeaconRepub)
}
#endif