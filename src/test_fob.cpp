/**

Copyright (c) 2016, Heller Florian, Meißner Pascal, Nguyen Trung, Yi Xie
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "ros/ros.h"
#include "asr_msgs/AsrObject.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

ros::Publisher pub_states;
sensor_msgs::JointState joint_state;

ros::Subscriber sub_tf;
tf::TransformBroadcaster *pub_tf;
geometry_msgs::TransformStamped odom_trans;

/**
 * Callback that receives the current pose of the tracker and publishes
 * the corresponding transformation for the hand model.
 */
void callback_tf(const asr_msgs::AsrObjectConstPtr& msg)
{
    if (msg->identifier == "tracker_right")
    {
        odom_trans.header.stamp = ros::Time::now();

        if(!msg->sampledPoses.size())
        {
            std::cerr << "Got a AsrObject without poses." << std::endl;
            std::exit(1);
        }

        geometry_msgs::Pose current_pose = msg->sampledPoses.front().pose;
        geometry_msgs::Point position = current_pose.position;

        odom_trans.transform.translation.x = position.x;
        odom_trans.transform.translation.y = position.y;
        odom_trans.transform.translation.z = position.z;
        odom_trans.transform.rotation = current_pose.orientation;

        pub_tf->sendTransform(odom_trans);

        // send 0-pose for hand model
        joint_state.header.stamp = ros::Time::now();
        pub_states.publish(joint_state);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_fob");
    ros::NodeHandle node;

    sub_tf = node.subscribe("fob_objects", 1, callback_tf);
    pub_tf = new tf::TransformBroadcaster();

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "fob_sensor";

    pub_states = node.advertise<sensor_msgs::JointState>("joint_states", 1);

    joint_state.name.resize(22);
    joint_state.position.resize(22);

    // the names are set accordingly to the names used in the xml model
    joint_state.name[0] ="wr_j0";
    joint_state.name[1] ="wr_j1";

    joint_state.name[2] ="th_j0";
    joint_state.name[3] ="th_j1";
    joint_state.name[4] ="th_j2";
    joint_state.name[5] ="th_j3";

    joint_state.name[6] ="in_j1";
    joint_state.name[7] ="in_j0";
    joint_state.name[8] ="in_j2";
    joint_state.name[9] ="in_j3";

    joint_state.name[10] ="mi_j1";
    joint_state.name[11] ="mi_j0";
    joint_state.name[12] ="mi_j2";
    joint_state.name[13] ="mi_j3";

    joint_state.name[14] ="ri_j1";
    joint_state.name[15] ="ri_j0";
    joint_state.name[16] ="ri_j2";
    joint_state.name[17] ="ri_j3";

    joint_state.name[18] ="pi_j1";
    joint_state.name[19] ="pi_j0";
    joint_state.name[20] ="pi_j2";
    joint_state.name[21] ="pi_j3";

    // 0-pose for hand model, since only the flock_of_birds tracker is tested
    joint_state.position[0] = 0;
    joint_state.position[1] = 0;
    joint_state.position[2] = 0;
    joint_state.position[3] = 0;
    joint_state.position[4] = 0;
    joint_state.position[5] = 0;
    joint_state.position[6] = 0;
    joint_state.position[7] = 0;
    joint_state.position[8] = 0;
    joint_state.position[9] = 0;
    joint_state.position[10] = 0;
    joint_state.position[11] = 0;
    joint_state.position[12] = 0;
    joint_state.position[13] = 0;
    joint_state.position[14] = 0;
    joint_state.position[15] = 0;
    joint_state.position[16] = 0;
    joint_state.position[17] = 0;
    joint_state.position[18] = 0;
    joint_state.position[19] = 0;
    joint_state.position[20] = 0;
    joint_state.position[21] = 0;

    ros::spin();

    return 0;
}
