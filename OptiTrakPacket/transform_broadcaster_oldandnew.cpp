#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
//#include "tf/transform_listener.h"
#include <tf2_ros/transform_broadcaster.h>



namespace tf {

TransformBroadcaster::TransformBroadcaster():
  tf2_broadcaster_()
{
};

void TransformBroadcaster::sendTransform(const geometry_msgs::TransformStamped & msgtf)
{
  tf2_broadcaster_.sendTransform(msgtf);
}

void TransformBroadcaster::sendTransform(const StampedTransform & transform)
{
  geometry_msgs::TransformStamped msgtf;
  transformStampedTFToMsg(transform, msgtf);
  tf2_broadcaster_.sendTransform(msgtf);
} 

void TransformBroadcaster::sendTransform(const std::vector<geometry_msgs::TransformStamped> & msgtf)
{
  tf2_broadcaster_.sendTransform(msgtf);
}

void TransformBroadcaster::sendTransform(const std::vector<StampedTransform> & transforms)
{
  std::vector<geometry_msgs::TransformStamped> msgtfs;
  for (std::vector<StampedTransform>::const_iterator it = transforms.begin(); it != transforms.end(); ++it)
  {
    geometry_msgs::TransformStamped msgtf;
    transformStampedTFToMsg(*it, msgtf);
    msgtfs.push_back(msgtf);

  }
  tf2_broadcaster_.sendTransform(msgtfs);
} 
  



}