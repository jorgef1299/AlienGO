#include "onboard_pc_locomotion_control.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "onboard_pc_locomotion_node");
    ros::NodeHandle n_public;

    Custom custom(HIGHLEVEL);
    custom.pub_high_state = n_public.advertise<AlienGo::HighState>("/AlienGo/high_state", 10);

    InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

}

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{
    udp.Send();
}

void Custom::RobotControl()
{
    if(motiontime <= 4500) {
        motiontime += 2;
        activateSportMode();
    }
    udp.GetRecv(state);
    // Fill ROS message with the current High State of the robot
    msg_high_state.levelFlag = state.levelFlag;
    msg_high_state.commVersion = state.commVersion;
    msg_high_state.robotID = state.robotID;
    msg_high_state.SN = state.SN;
    msg_high_state.bandWidth = state.bandWidth;
    msg_high_state.mode = state.mode;
    msg_high_state.imu.quaternion[0] = state.imu.quaternion[0];
    msg_high_state.imu.quaternion[1] = state.imu.quaternion[1];
    msg_high_state.imu.quaternion[2] = state.imu.quaternion[2];
    msg_high_state.imu.quaternion[3] = state.imu.quaternion[3];
    msg_high_state.imu.accelerometer[0] = state.imu.accelerometer[0];
    msg_high_state.imu.accelerometer[1] = state.imu.accelerometer[1];
    msg_high_state.imu.accelerometer[2] = state.imu.accelerometer[2];
    msg_high_state.imu.gyroscope[0] = state.imu.gyroscope[0];
    msg_high_state.imu.gyroscope[1] = state.imu.gyroscope[1];
    msg_high_state.imu.gyroscope[2] = state.imu.gyroscope[2];
    msg_high_state.imu.rpy[0] = state.imu.rpy[0];
    msg_high_state.imu.rpy[1] = state.imu.rpy[1];
    msg_high_state.imu.rpy[2] = state.imu.rpy[2];
    msg_high_state.imu.temperature = state.imu.temperature;
    msg_high_state.velocity[0] = state.velocity[0];
    msg_high_state.velocity[1] = state.velocity[1];
    msg_high_state.velocity[2] = state.velocity[2];
    msg_high_state.yawSpeed = state.yawSpeed;
    for(uint8_t i=0; i < 4; i++) {
        msg_high_state.footPosition2Body[i].x = state.footPosition2Body[i].x;
        msg_high_state.footPosition2Body[i].y = state.footPosition2Body[i].y;
        msg_high_state.footPosition2Body[i].z = state.footPosition2Body[i].z;
    }
    for(uint8_t i=0; i < 4; i++) {
        msg_high_state.footSpeed2Body[i].x = state.footSpeed2Body[i].x;
        msg_high_state.footSpeed2Body[i].y = state.footSpeed2Body[i].y;
        msg_high_state.footSpeed2Body[i].z = state.footSpeed2Body[i].z;
    }
    for(uint8_t i=0; i < 4; i++) {
        msg_high_state.footForce[i] = state.footForce[i];
    }
    for(uint8_t i=0; i < 40; i++) {
        msg_high_state.wirelessRemote[i] = state.wirelessRemote[i];
    }
    msg_high_state.reserve = state.reserve;
    msg_high_state.crc = state.crc;
    // ROS: Publish current high state
    pub_high_state.publish(msg_high_state);
}

void Custom::activateSportMode() {
    if (motiontime == 2)
    {
        std::cout<<"Begin sending commands."<<std::endl;
    }
    if (motiontime>10 && motiontime <100)
    {
        cmd = {0};
        cmd.levelFlag = 0xf0;
        udp.SetSend(cmd);
    }
    else if (motiontime == 100)
    {
        std::cout<<"Aliengo sport mode trigger sent !"<<std::endl;

    }
    else if (motiontime >= 4000 && motiontime <4500)
    {
        cmd = {0};
        cmd.mode = 1; // to force stand status
        udp.SetSend(cmd);
    }
    else if (motiontime == 4500)
    {
        std::cout<<"Finshed ! Robot in stand. "<<std::endl;
    }
    else if (motiontime > 4500)
    {
        cmd = {0};
        cmd.mode = 0;
        cmd.gaitType = 0;
        udp.SetSend(cmd);
    }
}

