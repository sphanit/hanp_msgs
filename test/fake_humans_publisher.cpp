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

#define START_POSE_X 0.0
#define START_POSE_Y 0.0
#define END_POSE_X 4.0
#define END_POSE_Y 4.0
#define LOOP_RATE 10

#include <ros/ros.h>
#include <hanp_msgs/TrackedHumans.h>
#include <visualization_msgs/Marker.h>

#include <math.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_human_publisher");

    // ros variables
    ros::NodeHandle nh("~");
    ros::Publisher humans_pub, vis_pub;

    // variables
    bool publish_markers = false;

    // get parameters from server
    nh.param<bool>("publish_markers", publish_markers, false);

    humans_pub = nh.advertise<hanp_msgs::TrackedHumans>("humans", 1);

    if(publish_markers)
    {
        vis_pub = nh.advertise<visualization_msgs::Marker>( "humans_marker", 0);
        ROS_DEBUG("will publish markers");
    }

    ros::Rate loop_rate(LOOP_RATE);

    // generate fake humans and markers
    hanp_msgs::TrackedHuman human;
    visualization_msgs::Marker human_marker;
    geometry_msgs::Pose start_pose, end_pose;

    start_pose.position.x = START_POSE_X;
    start_pose.position.y = START_POSE_Y;
    start_pose.position.z = 0.0;
    start_pose.orientation.x = 0.0;
    start_pose.orientation.y = 0.0;
    start_pose.orientation.z = 0.0;
    start_pose.orientation.w = 1.0;

    end_pose = start_pose;
    end_pose.position.x = END_POSE_X;
    end_pose.position.y = END_POSE_Y;

    double diff_x = end_pose.position.x - start_pose.position.x;
    double diff_y = end_pose.position.y - start_pose.position.y;

    human.track_id = 1;
    human.pose.pose = start_pose;

    // add human to humans message to publish
    hanp_msgs::TrackedHumans humans;
    humans.header.stamp = ros::Time::now();
    humans.header.frame_id = "humans_frame";
    humans.tracks.push_back(human);

    // add marker for the fake human for visualization
    human_marker.header.stamp = humans.header.stamp;
    human_marker.header.frame_id = humans.header.frame_id;
    human_marker.type = visualization_msgs::Marker::ARROW;
    human_marker.action = visualization_msgs::Marker::MODIFY;
    human_marker.id = human.track_id;
    human_marker.pose = human.pose.pose;
    human_marker.scale.x = 0.5;
    human_marker.scale.y = 0.08;
    human_marker.scale.z = 0.08;
    human_marker.color.a = 1.0;
    human_marker.color.r = 0.5;
    human_marker.color.g = 0.5;
    human_marker.color.b = 0.0;

    double angle = 0;
    while (ros::ok())
    {
        // change human pose by simple sin-wave based interpolation
        double interpolation = (sin(angle) + 1) / 2;
        human.pose.pose.position.x = start_pose.position.x + diff_x * interpolation;
        human.pose.pose.position.y = start_pose.position.y + diff_y * interpolation;

        // modify messages to publish
        humans.header.stamp = ros::Time::now();
        humans.tracks.clear();
        humans.tracks.push_back(human);
        human_marker.pose = human.pose.pose;

        humans_pub.publish(humans);
        if(publish_markers)
        {
            vis_pub.publish(human_marker);
        }

        ros::spinOnce();

        // reduce movement by 10th of the loop rate
        angle += M_PI/LOOP_RATE/10;
        loop_rate.sleep();
    }

    return 0;
}
