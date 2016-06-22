/*/
 * Copyright (c) 2015 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *                                  Harmish Khambhaita on Fri Jun 12 2015
 */

#define LOOP_RATE 10.0
#define POSE_UPDATE_RATE 20.0
#define SEGMENT_TYPE hanp_msgs::TrackedSegmentType::TORSO
#define HUMANS_FRAME_ID "humans_frame"
#define PUBLISH_MARKERS false
#define RELATIVE_ORIENTATION true
#define MARKER_LIFETIME 4.0

#include <ros/ros.h>
#include <hanp_msgs/TrackedHumans.h>
#include <hanp_msgs/TrackedSegmentType.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

#include <math.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_human_publisher");

    // ros variables
    ros::NodeHandle nh("~");
    ros::Publisher humans_pub, vis_pub;
    std::map<int, double> start_poses_x, start_poses_y, start_poses_theta, end_poses_x, end_poses_y, end_poses_theta;

    // other variables
    bool publish_markers, relative_orientation;

    // get parameters from server
    nh.param<bool>("publish_markers", publish_markers, PUBLISH_MARKERS);
    nh.param<bool>("relative_orientation", relative_orientation, RELATIVE_ORIENTATION);
    XmlRpc::XmlRpcValue human_positions;
    if(!nh.getParam("humans", human_positions))
    {
        ROS_ERROR("failed to read human positoins on param server");
    }

    // check and process human positions data
    enum class PoseType : char { START = 1, END = 2, LAST = 3 };

    std::map<int, std::map<PoseType, geometry_msgs::Pose>> humans_poses;
    ROS_ASSERT(human_positions.getType()==XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < human_positions.size(); ++i)
    {
        ROS_ASSERT(human_positions[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);

        bool got_id = false, got_start = false, got_end = false;
        int human_id;
        double human_start_x, human_start_y, human_start_theta, human_end_x, human_end_y, human_end_theta;

        for (XmlRpc::XmlRpcValue::iterator it = human_positions[i].begin(); it != human_positions[i].end(); ++it)
        {
            if(!it->first.compare("id"))
            {
                ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeInt);
                human_id = (int)(it->second);
                got_id = true;
            }
            else if(!it->first.compare("start"))
            {
                ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeArray);
                ROS_ASSERT(it->second.size() == 3);
                for (int32_t j = 0; j < it->second.size(); ++j)
                {
                    ROS_ASSERT(it->second[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                }
                human_start_x = (double)(it->second[0]);
                human_start_y = (double)(it->second[1]);
                human_start_theta = (double)(it->second[2]);
                got_start = true;
            }
            else if(!it->first.compare("end"))
            {
                ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeArray);
                ROS_ASSERT(it->second.size() == 3);
                for (int j = 0; j < it->second.size(); ++j)
                {
                    ROS_ASSERT(it->second[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                }
                human_end_x = (double)(it->second[0]);
                human_end_y = (double)(it->second[1]);
                human_end_theta = (double)(it->second[2]);
                got_end = true;
            }
            else
            {
                ROS_ERROR("could not process %s identifier for human positions", ((std::string)(it->first)).c_str());
                exit;
            }
        }

        if(!got_id || !got_start || !got_end)
        {
            ROS_ERROR("incomplete or wrong human position data");
            exit;
        }

        geometry_msgs::Pose start_pose, end_pose;

        start_pose.position.x = human_start_x;
        start_pose.position.y = human_start_y;
        start_pose.position.z = 0.0;
        start_pose.orientation = tf::createQuaternionMsgFromYaw(human_start_theta);

        end_pose.position.x = human_end_x;
        end_pose.position.y = human_end_y;
        end_pose.position.z = 0.0;
        end_pose.orientation = tf::createQuaternionMsgFromYaw(human_end_theta);

        std::map<PoseType, geometry_msgs::Pose> start_end_last_poses;
        start_end_last_poses[PoseType::START] = start_pose;
        start_end_last_poses[PoseType::END] = end_pose;
        start_end_last_poses[PoseType::LAST] = start_pose;

        humans_poses[human_id] = start_end_last_poses;
    }

    humans_pub = nh.advertise<hanp_msgs::TrackedHumans>("humans", 1);

    if(publish_markers)
    {
        vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "humans_markers", 0);
        ROS_DEBUG("will publish markers");
    }

    ros::Rate loop_rate(LOOP_RATE);

    // generate fake humans and markers
    hanp_msgs::TrackedHumans tracks;
    visualization_msgs::MarkerArray humans_markers;

    tracks.header.stamp = ros::Time::now();
    tracks.header.frame_id = HUMANS_FRAME_ID;

    std::map<int, double> diff_xs, diff_ys, diff_thetas;
    for(auto human_poses : humans_poses)
    {
        diff_xs[human_poses.first] = human_poses.second[PoseType::END].position.x - human_poses.second[PoseType::START].position.x;
        diff_ys[human_poses.first] = human_poses.second[PoseType::END].position.y - human_poses.second[PoseType::START].position.y;
        diff_thetas[human_poses.first] = tf::getYaw(human_poses.second[PoseType::END].orientation) - tf::getYaw(human_poses.second[PoseType::START].orientation);

        hanp_msgs::TrackedSegment tracked_segment;
        tracked_segment.type = SEGMENT_TYPE;
        tracked_segment.pose.pose = human_poses.second[PoseType::START];

        hanp_msgs::TrackedHuman human;
        human.track_id = human_poses.first;
        human.segments.push_back(tracked_segment);

        tracks.humans.push_back(human);

        visualization_msgs::Marker human_marker;
        human_marker.header.stamp = tracks.header.stamp;
        human_marker.header.frame_id = tracks.header.frame_id;
        human_marker.type = visualization_msgs::Marker::ARROW;
        human_marker.action = visualization_msgs::Marker::MODIFY;
        human_marker.id = human.track_id;
        human_marker.pose = tracked_segment.pose.pose;
        human_marker.scale.x = 0.5;
        human_marker.scale.y = 0.08;
        human_marker.scale.z = 0.08;
        human_marker.color.a = 1.0;
        human_marker.color.r = 0.5;
        human_marker.color.g = 0.5;
        human_marker.color.b = 0.0;
        human_marker.lifetime = ros::Duration(MARKER_LIFETIME);
        humans_markers.markers.push_back(human_marker);
    }

    double angle = 0;
    while (ros::ok())
    {
        // change human pose by simple sin-wave based interpolation
        double interpolation = (sin(angle) + 1) / 2;

        tracks.header.stamp = ros::Time::now();
        for(auto &human : tracks.humans)
        {
            for(auto &segment : human.segments)
            {
                if(segment.type == SEGMENT_TYPE)
                {
                    segment.pose.pose.position.x = humans_poses[human.track_id][PoseType::START].position.x
                        + diff_xs[human.track_id] * interpolation;
                    segment.pose.pose.position.y = humans_poses[human.track_id][PoseType::START].position.y
                        + diff_ys[human.track_id] * interpolation;

                    double dx = segment.pose.pose.position.x - humans_poses[human.track_id][PoseType::LAST].position.x;
                    double dy = segment.pose.pose.position.y - humans_poses[human.track_id][PoseType::LAST].position.y;

                    segment.twist.twist.linear.x = dx / (1.0/LOOP_RATE);
                    segment.twist.twist.linear.y = dy / (1.0/LOOP_RATE);

                    if(relative_orientation)
                    {
                        segment.pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(dy, dx));
                        segment.twist.twist.angular.z = (tf::getYaw(segment.pose.pose.orientation) -
                            tf::getYaw(humans_poses[human.track_id][PoseType::LAST].orientation)) / (1.0/LOOP_RATE);
                    }
                    else
                    {
                        segment.pose.pose.orientation = tf::createQuaternionMsgFromYaw(
                            tf::getYaw(humans_poses[human.track_id][PoseType::START].orientation)
                            + diff_thetas[human.track_id] * interpolation);

                        double dtheta = tf::getYaw(segment.pose.pose.orientation) - tf::getYaw(humans_poses[human.track_id][PoseType::LAST].orientation);
                        segment.twist.twist.angular.z = dtheta / (1.0/LOOP_RATE);
                    }

                    humans_poses[human.track_id][PoseType::LAST] = segment.pose.pose;

                    ROS_DEBUG("updated human %lu, x=%f, y=%f, theta=%f", human.track_id,
                        segment.pose.pose.position.x, segment.pose.pose.position.y,
                        tf::getYaw(segment.pose.pose.orientation));

                    if(publish_markers)
                    {
                        for(auto &human_marker : humans_markers.markers)
                        {
                            if(human_marker.id == human.track_id)
                            {
                                human_marker.pose = segment.pose.pose;
                            }
                        }
                    }
                }
            }
        }

        humans_pub.publish(tracks);
        if(publish_markers)
        {
            vis_pub.publish(humans_markers);
        }

        ros::spinOnce();

        // reduce movement by 10th of the loop rate
        angle += M_PI/LOOP_RATE/POSE_UPDATE_RATE;
        loop_rate.sleep();
    }

    return 0;
}
