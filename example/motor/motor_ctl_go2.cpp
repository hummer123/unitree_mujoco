#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unistd.h>

#include "motor_cmd_parse.hpp"


using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"


constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

class Custom
{
public:
    Custom(){};
    ~Custom(){};
    void Init();
    void onStand();
    void motor_control(int motorId, float q, float dq=0.0f, float kp=0.0f, float kd=0.0f, float tau=0.0f);

private:
    void InitLowCmd();
    void LowCmdMessageHandler(const void *messages);
  
private:
    double stand_up_joint_pos[12] = {0.00571868, 0.608813, -1.21763,  -0.00571868, 0.608813, -1.21763,
                                     0.00571868, 0.608813, -1.21763,  -0.00571868, 0.608813, -1.21763};
    double stand_down_joint_pos[12] = {0.0473455, 1.22187, -2.44375,  -0.0473455, 1.22187, -2.44375, 
                                       0.0473455, 1.22187, -2.44375,  -0.0473455, 1.22187, -2.44375};
    double dt = 0.002;
    double runing_time = 0.0;
    double phase = 0.0;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};     // default init
    unitree_go::msg::dds_::LowCmd_ sub_low_cmd{}; // for debug

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_subscriber;
};

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void Custom::Init()
{
    InitLowCmd();
    /*create publisher*/
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowcmd_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_subscriber->InitChannel(std::bind(&Custom::LowCmdMessageHandler, this, std::placeholders::_1), 1);
}



void Custom::motor_control(int motorId, float q, float dq, float kp, float kd, float tau)
{
    int target_motor = 0; // 目标电机ID (0-11)

    // 确保电机ID在有效范围内
    target_motor = std::max(0, std::min(11, motorId));

    low_cmd.motor_cmd()[target_motor].q() = q;
    low_cmd.motor_cmd()[target_motor].dq() = dq;
    low_cmd.motor_cmd()[target_motor].kp() = kp;
    low_cmd.motor_cmd()[target_motor].kd() = kd;
    low_cmd.motor_cmd()[target_motor].tau() = tau;
    std::cout << "Motor[" << target_motor << "] q: " << q << ", dq: " << dq << ", kp: " << kp << ", kd: " << kd << ", tau: " << tau << std::endl;

    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd);
}

void Custom::onStand()
{
    for (double t = 0; t < 3.0; t += dt)
    {
        phase = tanh(t / 1.2);
        for (int i = 0; i < 12; i++)
        {
            low_cmd.motor_cmd()[i].q() = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i];
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = phase * 50.0 + (1 - phase) * 20.0;
            low_cmd.motor_cmd()[i].kd() = 3.5;
            low_cmd.motor_cmd()[i].tau() = 0;
        }

        low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
        lowcmd_publisher->Write(low_cmd);

        usleep(static_cast<useconds_t>(500));
    }
    std::cout << "\nStand up completed." << std::endl;
}

void Custom::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01); // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}


void Custom::LowCmdMessageHandler(const void *message)
{
    // Detect if anyone else is sending 'LowCmd' command
    sub_low_cmd = *(unitree_go::msg::dds_::LowCmd_ *)message;
    // std::cout << "==> [LowCmd received] Motor[0] target pos: " << sub_low_cmd.motor_cmd()[0].q() << std::endl;
}


int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        ChannelFactory::Instance()->Init(1, "lo");
    }
    else
    {
        ChannelFactory::Instance()->Init(0, argv[1]);
    }
    std::cout << "Press enter to start. Type 'q' then Enter to quit.";
    std::cin.get();
    Custom custom;
    custom.Init();
    usleep(static_cast<useconds_t>(1 * 1e6));
    custom.onStand();

    std::string line;
    MotorCmdParse cmdParser;
    
    std::cout << "\n=== cmd Info ===" << std::endl;
    std::cout << cmdParser.PrintHelp() << std::endl;
    std::cout << "========================" << std::endl;

    while (true)
    {
        std::cout << "\nEntry cmd: ";
        std::getline(std::cin, line);
        if (!std::cin)
            break;
        if (line.size() > 0 && line[0] == 'q')
            break;

        auto cmd = cmdParser.ParseFromString(line);
        if (cmd.valid) {
            custom.motor_control(cmd.motor_id, cmd.q, cmd.dq, cmd.kp, cmd.kd, cmd.tau);
        } else {
            std::cerr << "Failed to parse command: " << cmd.error_msg << std::endl;
            std::cout << cmdParser.PrintHelp() << std::endl;
        }
    }

    return 0;
}
