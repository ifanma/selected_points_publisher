/*
 * Copyright 2019-2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *  v0.1.0: drwnz (david.wong@tier4.jp) *
 *
 * selected_points_publisher.cpp
 *
 *  Created on: December 5th 2019
 */

#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/view_manager.h"
#include "rviz/view_controller.h"
#include "OGRE/OgreCamera.h"

#include "selected_points_publisher/selected_points_publisher.hpp"

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <QVariant>
#include <visualization_msgs/Marker.h>

namespace rviz_plugin_selected_points_publisher
{
    SelectedPointsPublisher::SelectedPointsPublisher()
    {
        updateTopic();
    }

    SelectedPointsPublisher::~SelectedPointsPublisher()
    {
    }

    void SelectedPointsPublisher::updateTopic()
    {
        node_handle_.param("frame_id", tf_frame_, std::string("/base_link"));
        rviz_cloud_topic_ = std::string("/rviz_selected_points");

        rviz_selected_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(rviz_cloud_topic_.c_str(), 1);
        num_selected_points_ = 0;
    }

    int SelectedPointsPublisher::processKeyEvent(QKeyEvent *event, rviz::RenderPanel *panel)
    {
        if (event->type() == QKeyEvent::KeyPress)
        {
            if (event->key() == 'c' || event->key() == 'C')
            {
                ROS_INFO_STREAM_NAMED("SelectedPointsPublisher::processKeyEvent", "Cleaning previous selection (selected area "
                                                                                  "and points).");
                rviz::SelectionManager *selection_manager = context_->getSelectionManager();
                rviz::M_Picked selection = selection_manager->getSelection();
                selection_manager->removeSelection(selection);
                visualization_msgs::Marker marker;
                // Set the frame ID and timestamp.  See the TF tutorials for information on these.
                marker.header.frame_id = context_->getFixedFrame().toStdString().c_str();
                marker.header.stamp = ros::Time::now();
                marker.ns = "basic_shapes";
                marker.id = 0;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::DELETE;
                marker.lifetime = ros::Duration();
                num_selected_points_ = 0;
            }
            else if (event->key() == 'p' || event->key() == 'P')
            {
                ROS_INFO_STREAM_NAMED("SelectedPointsPublisher.updateTopic",
                                      "Publishing " << num_selected_points_ << " selected points to topic "
                                                    << node_handle_.resolveName(rviz_cloud_topic_));
                if (selected_points_.data.size() > 0)
                    rviz_selected_publisher_.publish(selected_points_);
            }
        }
    }

    int SelectedPointsPublisher::processMouseEvent(rviz::ViewportMouseEvent &event)
    {
        int flags = rviz::SelectionTool::processMouseEvent(event);
        if (event.alt())
        {
            selecting_ = false;
        }
        else
        {
            if (event.leftDown())
            {
                selecting_ = true;
            }
        }

        if (selecting_)
        {
            if (event.leftUp())
            {
                this->processSelectedArea();
            }
        }
        return flags;
    }

    int SelectedPointsPublisher::processSelectedArea()
    {
        rviz::SelectionManager *selection_manager = context_->getSelectionManager();
        rviz::M_Picked selection = selection_manager->getSelection();
        rviz::PropertyTreeModel *model = selection_manager->getPropertyModel();

        selected_points_.header.frame_id = context_->getFixedFrame().toStdString();
        selected_points_.height = 1;
        // selected_points_.point_step = 4 * 4;
        selected_points_.is_dense = false;
        selected_points_.is_bigendian = false;

        selected_points_.fields.resize(3);

        selected_points_.fields[0].name = "x";
        selected_points_.fields[0].offset = 0;
        selected_points_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        selected_points_.fields[0].count = 1;

        selected_points_.fields[1].name = "y";
        selected_points_.fields[1].offset = 4;
        selected_points_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        selected_points_.fields[1].count = 1;

        selected_points_.fields[2].name = "z";
        selected_points_.fields[2].offset = 8;
        selected_points_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        selected_points_.fields[2].count = 1;

        if (model->hasIndex(0, 0))
        {
            ROS_INFO_STREAM_NAMED("SelectedPointsPublisher._processSelectedAreaAndFindPoints",
                                  "Getting fields: ");
            auto index = model->index(0, 0);
            rviz::Property *child = model->getProp(index);
            int offset = 12;

            // 从配置中找出别的数据，加在点云坐标数据的后面，childAt(0)是position
            for (int j = 1; j < child->numChildren(); j++)
            {
                rviz::Property *grandchild = child->childAt(j);

                sensor_msgs::PointField field = selected_points_.fields[2]; //拙略的把position.z的field赋为初值

                ROS_INFO_STREAM(grandchild->getName().split(": ").last().toStdString() << " " << selected_points_.fields[2].name);
                field.name = grandchild->getName().split(": ").last().toStdString();
                field.offset = offset;
                field.datatype = sensor_msgs::PointField::UINT32;
                selected_points_.fields.push_back(field);
                offset += 4; // 就是数据类型是uint32，是四个字节，核心是在处理sensor_msgs::PointCloud2话题数据所需要的内容

                std::stringstream ss;
                ss << field.name << " "
                   << " " << grandchild->getValue().typeName()
                   << " " << grandchild->getValue().toFloat();

                ROS_INFO_STREAM_NAMED("SelectedPointsPublisher._processSelectedAreaAndFindPoints",
                                      ss.str());
            }
            selected_points_.point_step = offset;
        }
        else
        {
            ROS_INFO_STREAM_NAMED("SelectedPointsPublisher._processSelectedAreaAndFindPoints",
                                  "No point to send: " << num_selected_points_);
            selected_points_ = sensor_msgs::PointCloud2();
        }

        int i = 0;
        int num_children = model->children().size();

        ROS_INFO_STREAM("num_children: "<< " "<< num_children);

        selected_points_.data.clear();
        selected_points_.data.reserve(num_children * selected_points_.point_step);
        while (model->hasIndex(i, 0))
        {
            selected_points_.row_step = (i + 1) * selected_points_.point_step;
            selected_points_.data.resize(selected_points_.row_step);

            QModelIndex child_index = model->index(i, 0);

            rviz::Property *child = model->getProp(child_index);
            rviz::VectorProperty *subchild = (rviz::VectorProperty *)child->childAt(0);
            Ogre::Vector3 point_data = subchild->getVector();

            // 取出选择区域变量的首地址叠加偏移，然后赋值Point部分
            uint8_t *data_pointer = &selected_points_.data[0] + i * selected_points_.point_step;        
            *(float *)data_pointer = point_data.x;
            data_pointer += 4;
            *(float *)data_pointer = point_data.y;
            data_pointer += 4;
            *(float *)data_pointer = point_data.z;
            data_pointer += 4;

            // populate the fields
            for (int j = 1; j < child->numChildren(); j++)
            {
                rviz::Property *grandchild = child->childAt(j);
                uint32_t field = grandchild->getValue().toUInt();
                *(uint32_t *)data_pointer = field;      // 赋值field部分
                data_pointer += 4;
            }
            i++;
        }
        num_selected_points_ = i;
        ROS_INFO_STREAM_NAMED("SelectedPointsPublisher._processSelectedAreaAndFindPoints",
                              "Number of points in the selected area: " << num_selected_points_);
        ROS_INFO_STREAM_NAMED("SelectedPointsPublisher._processSelectedAreaAndFindPoints",
                              "Press 'p' to publish, or 'c' to clear");

        selected_points_.width = i;
        selected_points_.header.stamp = ros::Time::now();
        return 0;
    }
} // namespace rviz_plugin_selected_points_publisher

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_selected_points_publisher::SelectedPointsPublisher, rviz::Tool)
