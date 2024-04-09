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

#ifndef SONAR_DISPLAY_H
#define SONAR_DISPLAY_H

#ifndef Q_MOC_RUN
//#include <boost/circular_buffer.hpp>

#include <rviz/message_filter_display.h>
#include <imagenex831l/ProcessedRange.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "sensor_msgs/PointCloud2.h"

#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class IntProperty;
class PointCloudCommon;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace sonar_rviz_plugin
{



class SonarDisplay: public rviz::MessageFilterDisplay<imagenex831l::ProcessedRange>
{
Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  SonarDisplay();
  virtual ~SonarDisplay();

  virtual void update( float wall_dt, float ros_dt );

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();

  // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
  //void updateColorAndAlpha();
  void updateQueueSize();

  // Function to handle an incoming ROS message.
private:
  void processMessage( const imagenex831l::ProcessedRange::ConstPtr& scan );

  // Storage for the list of visuals.  It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  //boost::circular_buffer<boost::shared_ptr<ImuVisual> > visuals_;

  boost::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  boost::shared_ptr<tf2_ros::TransformListener> tfListener_;

  //pcl::PointCloud<pcl::PointXYZI>::Ptr allPointCloud_;
  sensor_msgs::PointCloud2Ptr allPointCloud_;

  // User-editable property variables.
  rviz::IntProperty* queue_size_property_;
  rviz::PointCloudCommon* point_cloud_common_;


};
// END_TUTORIAL

} // end namespace sonar_rviz_plugin

#endif // SONAR_DISPLAY_H
// %EndTag(FULL_SOURCE)%
