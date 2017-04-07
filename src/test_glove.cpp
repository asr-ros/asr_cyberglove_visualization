/**

Copyright (c) 2016, Heller Florian, Jäkel Rainer, Kasper Alexander, Meißner Pascal, Nguyen Trung, Yi Xie
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "ros/ros.h"
#include "asr_msgs/AsrGlove.h"
#include "asr_msgs/AsrObject.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

ros::Subscriber sub_states;
ros::Publisher pub_states;
sensor_msgs::JointState joint_state;

/**
 * Callback that receives the current joint state from the sensors of the
 * right glove and publishes them for the hand model.
 */
void callback(const asr_msgs::AsrGloveConstPtr& msg)
{
    joint_state.header.stamp = ros::Time::now();

    // wrist: not needed because used in combination with asr_flock_of_birds
    joint_state.position[0] = 0;
    joint_state.position[1] = 0;

    // some values are manually adjusted since its difficult to calibrate
    // TODO: correct mapping of thumb joints
    joint_state.position[2] = 0.17;
    joint_state.position[3] = 0.02;
    joint_state.position[4] = 0.17;
    joint_state.position[5] = msg->data[5];

    joint_state.position[6] = msg->data[6];
    joint_state.position[7] = msg->data[7];
    joint_state.position[8] = msg->data[8];
    joint_state.position[9] = msg->data[9] * (-1);

    joint_state.position[10] = msg->data[10] - 0.1;
    joint_state.position[11] = msg->data[11];
    joint_state.position[12] = msg->data[12];
    joint_state.position[13] = msg->data[13] - 0.5;

    joint_state.position[14] = msg->data[14];
    joint_state.position[15] = msg->data[15] - 0.15;
    joint_state.position[16] = msg->data[16];
    joint_state.position[17] = msg->data[17] - 0.5;

    joint_state.position[18] = msg->data[18] - 0.3;
    joint_state.position[19] = msg->data[19] - 0.3;
    joint_state.position[20] = msg->data[20];
    joint_state.position[21] = msg->data[21] * (-1) + 0.1;

    pub_states.publish(joint_state);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_glove");
    ros::NodeHandle node;

    sub_states = node.subscribe("rightGloveData_radian", 1, callback);
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

    ros::spin();

    return 0;
}
