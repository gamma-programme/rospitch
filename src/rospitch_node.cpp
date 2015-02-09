// ROS C++ Headers
#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Quaternion.h>

#include <mavros/Waypoint.h>
#include <mavros/WaypointList.h>

// Pitch Headers
#include <RTI/RTI1516.h>
#include <RTI/RTIambassadorFactory.h>
#include <RTI/NullFederateAmbassador.h>
#include <RTI/time/HLAinteger64TimeFactory.h>
#include <RTI/time/HLAinteger64Time.h>
#include <RTI/time/HLAinteger64Interval.h>
#include <RTI/encoding/BasicDataElements.h>

/**
Initialise the connection to pitch
*/

void init_pitch(std::string nodeName, std::wstring connectionURI )
{
  std::auto_ptr<rti1516e::RTIambassadorFactory> rtiAmbassadorFactory(new rti1516e::RTIambassadorFactory());
  std::auto_ptr<rti1516e::RTIambassador> rtiAmbassador = rtiAmbassadorFactory->createRTIambassador();
  std::auto_ptr<rti1516e::NullFederateAmbassador> federateAmbassador(new rti1516e::NullFederateAmbassador());
  
  connectionURI = L"crcAddress=" + connectionURI;
  rtiAmbassador->connect(*federateAmbassador, rti1516e::HLA_IMMEDIATE, connectionURI);
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  ROS_INFO("GPS Latitude: [%f]", msg->latitude);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  geometry_msgs::Quaternion orientation = msg->orientation;
  ROS_INFO("Orientation: [%f]", orientation.x);
}

void waypointCallback(const mavros::WaypointList::ConstPtr& msg)
{
  std::vector<mavros::Waypoint> waypoints = msg->waypoints;
  
  std::vector<mavros::Waypoint>::iterator i = waypoints.begin();
  std::vector<mavros::Waypoint>::iterator end = waypoints.end();

  for(; i != end; i++){
    ROS_INFO("Waypoint [%i]: [%f], [%f]", (int) (i - waypoints.begin()), i->x_lat, i->y_long);
  }
}

std::wstring string_to_wstring(std::string s)
{
  std::wstring ws;
  ws.assign(s.begin(), s.end());
  return ws;
}

// Standard entry point
int main(int argc, char** argv) {
  // Announce this program to the ROS master, and add a random unique number to the name as we shouldn't have subscribers
  ros::init(argc, argv, "rospitch_node", ros::init_options::AnonymousName);
  
  // Start the node resource managers (communication, time, etc)
  ros::start();
  
  ROS_INFO_STREAM("Booting...");
  
  // ~ represents a private namespace, so internal parameters do not leak out
  ros::NodeHandle nh("~");
  
  std::vector<ros::Subscriber> subscriptions(5);
  
  subscriptions.push_back( nh.subscribe("gps/fix", 1000, gpsCallback) );
  subscriptions.push_back( nh.subscribe("imu/data", 1000, imuCallback) );
  subscriptions.push_back( nh.subscribe("mission/waypoints", 1000, waypointCallback) );
  
  ROS_INFO_STREAM("Connected to FCU");
  
  ROS_INFO_STREAM("Initiating connection to Federation Server...");
  
  std::string pitch_uri;
  
  nh.param<std::string>("pitch_uri", pitch_uri, "127.0.0.1");
  
  init_pitch(ros::this_node::getName(), string_to_wstring(pitch_uri));
  
  ROS_INFO_STREAM("Connected to Federation Server");
  
  // Process ROS callbacks until receiving a SIGINT (ctrl-c)
  ros::spin();
  
  // Stop the node's resources
  ros::shutdown();
  
  // Exit tranquilly
  return 0;
}