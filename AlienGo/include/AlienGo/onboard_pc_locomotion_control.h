#ifndef SRC_ONBOARD_PC_LOCOMOTION_CONTROL_H
#define SRC_ONBOARD_PC_LOCOMOTION_CONTROL_H
#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "unitree_legged_sdk.h"
#include "AlienGo/HighState.h"
#include "AlienGo/HighCmd.h"

using namespace UNITREE_LEGGED_SDK;

// High Cmd
constexpr uint16_t TARGET_PORT = 8082;
constexpr uint16_t LOCAL_PORT = 8081;
constexpr char TARGET_IP[] = "127.0.0.1";   // target IP address

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::Aliengo),
                           udp(LOCAL_PORT, TARGET_IP,TARGET_PORT, sizeof(HighCmd), sizeof(HighState))
    {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    void activateSportMode();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
    AlienGo::HighState msg_high_state;
    ros::Publisher pub_high_state;
};

#endif //SRC_ONBOARD_PC_LOCOMOTION_CONTROL_H
