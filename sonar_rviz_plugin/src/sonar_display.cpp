/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sonar_display.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

//#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <ros/time.h>
#include "rviz/default_plugin/point_cloud_common.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/point_cloud.h"
#include "rviz/properties/int_property.h"
#include "rviz/validate_floats.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/point_cloud2_iterator.h>



using namespace std;

namespace sonar_rviz_plugin
{

// BEGIN_TUTORIAL
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
SonarDisplay::SonarDisplay(): 
    point_cloud_common_( new rviz::PointCloudCommon( this )),
    allPointCloud_(new sensor_msgs::PointCloud2)
  
{

  //tfListener = new tf2_ros::TransformListener(tfBuffer);
  tfBuffer_.reset(new tf2_ros::Buffer());
  tfListener_.reset(new tf2_ros::TransformListener(*tfBuffer_));
  queue_size_property_ = new rviz::IntProperty( "Queue Size", 10,
                                          "Advanced: set the size of the incoming Range message queue. "
                                          " Increasing this is useful if your incoming TF data is delayed significantly "
                                          "from your LaserScan data, but it can greatly increase memory usage if the messages are big.",
                                          this, SLOT( updateQueueSize() ));
 
  queue_size_property_->setMin( 1 );
  queue_size_property_->setMax( 100000 );


  // PointCloudCommon sets up a callback queue with a thread for each
  // instance.  Use that for processing incoming messages.
  // update_nh_.setCallbackQueue( point_cloud_common_->getCallbackQueue() );

  ROS_INFO_STREAM("Sonar Constructor");
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void SonarDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateQueueSize();
  point_cloud_common_->initialize( context_, scene_node_ );
  ROS_INFO_STREAM("Sonar Initialized Successful!");
}

SonarDisplay::~SonarDisplay()
{
   delete point_cloud_common_;
}

// Clear the visuals by deleting their objects.
void SonarDisplay::reset()
{
  MFDClass::reset();
  point_cloud_common_->reset();
}


// Set the number of past visuals to show.
void SonarDisplay::updateQueueSize()
{
  tf_filter_->setQueueSize( (uint32_t) queue_size_property_->getInt() );
  ROS_INFO_STREAM("Sonar Update QueueSize!");
}

// This is our callback to handle an incoming message.
void SonarDisplay::processMessage( const imagenex831l::ProcessedRange::ConstPtr& scan )
{


  pcl::PointXYZI pclPoint;
  geometry_msgs::PointStamped geometryPoint;
  
  

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
  //sensor_msgs::PointCloud2 rosCloud;
  sensor_msgs::PointCloud2Ptr rosCloud( new sensor_msgs::PointCloud2 );
  sensor_msgs::PointCloud2Ptr rosCloud2( new sensor_msgs::PointCloud2 );


  // Cloud output in "odom"/target_frame
  sensor_msgs::PointCloud2Ptr rosCloud_out;//( new sensor_msgs::PointCloud2 );
  rosCloud_out.reset(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2Ptr rosCloud2_out( new sensor_msgs::PointCloud2 );

  double rangeResolution = scan->max_range / scan->intensity.size();


  
  int max = 0;
  int maxIndex = 0;
  for (int i=0; i<scan->intensity.size();i++){
    if (scan->intensity[i] > max){
       max = scan->intensity[i];
       maxIndex = i;
    }
  }
       
  geometryPoint.header = scan->header;
  geometryPoint.point.x = (maxIndex + 1) * rangeResolution * cos(( scan->head_position * M_PI ) / 180);
  geometryPoint.point.y = (maxIndex + 1) * rangeResolution * sin(( scan->head_position * M_PI ) / 180);
  geometryPoint.point.z = 0;
  
 
  pclPoint.x = geometryPoint.point.x ;
  pclPoint.y = geometryPoint.point.y;
  pclPoint.z = geometryPoint.point.z;
  pclPoint.intensity = scan->intensity[maxIndex];



  //if (allPointCloud_->size() > 360)
      //allPointCloud_->erase(allPointCloud_->begin() + 0);

  //We add the point to the cloud that later will be converted to the ROS message
  if (pclPoint.intensity > 10)
    cloud2->points.push_back(pclPoint); // TODO Parameter!


  pcl::toROSMsg(*cloud2, *rosCloud2);
  rosCloud2->header = scan->header;
  

  for (int i=0; i<scan->intensity.size(); i++){

    geometryPoint.header = scan->header;
    geometryPoint.point.x = (i + 1) * rangeResolution * cos(( scan->head_position * M_PI ) / 180);
    geometryPoint.point.y = (i + 1) * rangeResolution * sin( ( scan->head_position * M_PI ) / 180);
    geometryPoint.point.z = 0;

    pclPoint.x = geometryPoint.point.x ;
    pclPoint.y = geometryPoint.point.y;
    pclPoint.z = geometryPoint.point.z;
    pclPoint.intensity = scan->intensity[i];


    cloud->points.push_back(pclPoint);
    
  }

  pcl::toROSMsg(*cloud, *rosCloud);
  rosCloud->header = scan->header;

  try{
      //*rosCloud_out = tfBuffer_->transform(*rosCloud, "odom");
    tfBuffer_->transform(*rosCloud, *rosCloud_out, fixed_frame_.toStdString(), ros::Duration(0.1));
  }
  catch (tf2::TransformException &ex){
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
  }
  

  try{
      //rosCloud2_out = tfBuffer_->transform(rosCloud2, "odom");
    tfBuffer_->transform(*rosCloud2, *rosCloud2_out, fixed_frame_.toStdString(), ros::Duration(0.1));
  }
  catch (tf2::TransformException &ex){
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
  }

  //allPointCloud_->points.push_back(*rosCloud2_out);
  /*if (allPointCloud_->data.size() == 0)
  {
    allPointCloud_->header = rosCloud2_out->header;
    allPointCloud_->width = rosCloud2_out->width;
    allPointCloud_->height = rosCloud2_out->height;
    allPointCloud_->fields = rosCloud2_out->fields;
    allPointCloud_->is_bigendian = rosCloud2_out->is_bigendian;
    allPointCloud_->point_step = rosCloud2_out->point_step;
    allPointCloud_->row_step = rosCloud2_out->row_step;
    allPointCloud_->is_dense = rosCloud2_out->is_dense;
  }
  else
  {
    allPointCloud_->width += rosCloud2_out->width;
  }
  allPointCloud_->data.insert( allPointCloud_->data.end(), rosCloud2_out->data.begin(), rosCloud2_out->data.end() );
  */
  //ROS_ERROR_STREAM(*rosCloud2);
  //ROS_ERROR_STREAM(*rosCloud2_out);

  //point_cloud_common_->addMessage( rosCloud_out );
  //point_cloud_common_->addMessage( allPointCloud_ ); // Commented the permanent storing of point cloud.
  point_cloud_common_->addMessage(rosCloud2_out);

 
 
}


void SonarDisplay::update( float wall_dt, float ros_dt )
{
  point_cloud_common_->update( wall_dt, ros_dt );

  //ROS_INFO_STREAM("Sonar Point Cloud Update Successful!");
}


} // end namespace sonar_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sonar_rviz_plugin::SonarDisplay,rviz::Display )
// END_TUTORIAL
