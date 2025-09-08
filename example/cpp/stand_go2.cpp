#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unistd.h>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

class Custom
{
public:
    Custom(){};
    ~Custom(){};
    void Init();
    // Execute one full motion cycle (stand up then stand down)
    void onStand();
    void oneLegMotion(int motorId, double val);
    void oneLegMotion(double t, const std::string& command);

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void *messages);
    void LowCmdMessageHandler(const void *messages);
    void LowCmdWrite();
  
private:
    double stand_up_joint_pos[12] = {0.00571868, 0.608813, -1.21763,  -0.00571868, 0.608813, -1.21763,
                                     0.00571868, 0.608813, -1.21763,  -0.00571868, 0.608813, -1.21763};
    double stand_down_joint_pos[12] = {0.0473455, 1.22187, -2.44375,  -0.0473455, 1.22187, -2.44375, 
                                       0.0473455, 1.22187, -2.44375,  -0.0473455, 1.22187, -2.44375};
    double dt = 0.002;
    double runing_time = 0.0;
    double phase = 0.0;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};     // default init
    unitree_go::msg::dds_::LowState_ low_state{}; // default init
    unitree_go::msg::dds_::LowCmd_ sub_low_cmd{}; // for debug

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    // ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_subscriber;

    /*LowCmd write thread (unused when using DoMotion)*/
    ThreadPtr lowCmdWriteThreadPtr;
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
    // lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    // lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

    lowcmd_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_subscriber->InitChannel(std::bind(&Custom::LowCmdMessageHandler, this, std::placeholders::_1), 1);

    /* Note: do not start recurrent thread here. Use DoMotion() to perform one motion on demand. */
}



void Custom::oneLegMotion(int motorId, double val)
{
    double wave_line = 0.0;
    int control_mode = 0; // 0: 站立, 1: 蹲下
    int target_motor = 0; // 目标电机ID (0-11)

    control_mode = ((val > 0)? 0 : 1); // 默认自动模式
    wave_line = std::abs(val);

    // 确保电机ID在有效范围内
    target_motor = std::max(0, std::min(11, motorId));

    switch (control_mode) {
        case 0: // 站立模式
        {
            phase = tanh(wave_line / 1.2);
            // low_cmd.motor_cmd()[target_motor].q() = phase * stand_up_joint_pos[target_motor] + (1 - phase) * stand_down_joint_pos[target_motor];
            low_cmd.motor_cmd()[target_motor].q() = val;
            low_cmd.motor_cmd()[target_motor].dq() = 0;
            low_cmd.motor_cmd()[target_motor].kp() = phase * 50.0 + (1 - phase) * 20.0;
            low_cmd.motor_cmd()[target_motor].kd() = 3.5;
            low_cmd.motor_cmd()[target_motor].tau() = 0;
            std::cout << "Stand - Motor[" << target_motor << "] val:" << val << ", phase: " << phase << ", q: " << low_cmd.motor_cmd()[target_motor].q() 
                      << ", kp: " << low_cmd.motor_cmd()[target_motor].kp() << std::endl;
            break;
        }
        case 1: // 蹲下模式
        {
            phase = tanh(wave_line / 1.2);
            // low_cmd.motor_cmd()[target_motor].q() = phase * stand_down_joint_pos[target_motor] + (1 - phase) * stand_up_joint_pos[target_motor];
            low_cmd.motor_cmd()[target_motor].q() = 0;
            low_cmd.motor_cmd()[target_motor].dq() = val;
            low_cmd.motor_cmd()[target_motor].kp() = 50.0; // 高位置增益
            low_cmd.motor_cmd()[target_motor].kd() = 3.5;
            low_cmd.motor_cmd()[target_motor].tau() = 0;
            std::cout << "Down - Motor[" << target_motor << "] val:" << val << ", phase: " << phase << ", q: " << low_cmd.motor_cmd()[target_motor].q() 
                      << ", kp: " << low_cmd.motor_cmd()[target_motor].kp() << std::endl;
            break;
        }
    }

    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd);
}

void Custom::oneLegMotion(double t, const std::string& command)
{
    double wave_line = 0.0;
    int control_mode = 0; // 0: 站立, 1: 蹲下
    int target_motor = 0; // 目标电机ID (0-11)

    control_mode = ((t > 0)? 0 : 1); // 默认自动模式
    wave_line = std::abs(t);

    // 确保电机ID在有效范围内
    target_motor = std::max(0, std::min(11, target_motor));
    
    switch (control_mode) {
        case 0: // 站立模式
        {
            phase = tanh(wave_line / 1.2);
            for (int i = 0; i < 2; i++) {
                low_cmd.motor_cmd()[i].q() = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i];
                low_cmd.motor_cmd()[i].dq() = 0;
                low_cmd.motor_cmd()[i].kp() = phase * 50.0 + (1 - phase) * 20.0;
                low_cmd.motor_cmd()[i].kd() = 3.5;
                low_cmd.motor_cmd()[i].tau() = 0;
            }
            std::cout << "Stand - Motor[0] t:" << t << ", phase: " << phase << ", q: " << low_cmd.motor_cmd()[0].q() 
                      << ", kp: " << low_cmd.motor_cmd()[0].kp() << std::endl;
            break;
        }
        case 1: // 蹲下模式
        {
            phase = tanh(wave_line / 1.2);
            for (int i = 0; i < 2; i++) {
                low_cmd.motor_cmd()[i].q() = phase * stand_down_joint_pos[i] + (1 - phase) * stand_up_joint_pos[i];
                low_cmd.motor_cmd()[i].dq() = 0;
                low_cmd.motor_cmd()[i].kp() = 50.0; // 高位置增益
                low_cmd.motor_cmd()[i].kd() = 3.5;
                low_cmd.motor_cmd()[i].tau() = 0;
            }
            std::cout << "Down - Motor[0] t:" << t << ", phase: " << phase << ", q: " << low_cmd.motor_cmd()[0].q() 
                      << ", kp: " << low_cmd.motor_cmd()[0].kp() << std::endl;
            break;
        }
    }

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

void Custom::LowStateMessageHandler(const void *message)
{
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;
    // std::cout << "==> [LowState received] Motor[0] pos: " << low_state.motor_state()[0].q() << std::endl;
}

void Custom::LowCmdMessageHandler(const void *message)
{
    // Detect if anyone else is sending 'LowCmd' command
    sub_low_cmd = *(unitree_go::msg::dds_::LowCmd_ *)message;
    // std::cout << "==> [LowCmd received] Motor[0] target pos: " << sub_low_cmd.motor_cmd()[0].q() << std::endl;
}

void Custom::LowCmdWrite()
{
    runing_time += dt;
    if (runing_time < 3.0)
    {
        // Stand up in first 3 second

        // Total time for standing up or standing down is about 1.2s
        phase = tanh(runing_time / 1.2);
        for (int i = 0; i < 12; i++)
        {
            low_cmd.motor_cmd()[i].q() = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i];
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = phase * 50.0 + (1 - phase) * 20.0;
            low_cmd.motor_cmd()[i].kd() = 3.5;
            low_cmd.motor_cmd()[i].tau() = 0;
        }
    }
    else
    {
        // Then stand down
        phase = tanh((runing_time - 3.0) / 1.2);
        for (int i = 0; i < 12; i++)
        {
            low_cmd.motor_cmd()[i].q() = phase * stand_down_joint_pos[i] + (1 - phase) * stand_up_joint_pos[i];
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = 50;
            low_cmd.motor_cmd()[i].kd() = 3.5;
            low_cmd.motor_cmd()[i].tau() = 0;
        }
    }

    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd);
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
    double dt = 0.1;
    
    std::cout << "\n=== cmd Info ===" << std::endl;
    std::cout << "MotorId q, eg. 0 0.005" << std::endl;
    std::cout << "========================" << std::endl;

    while (true)
    {
        std::cout << "\nEntry cmd: ";
        std::getline(std::cin, line);
        if (!std::cin)
            break;
        if (line.size() > 0 && line[0] == 'q')
            break;

        // 解析输入并执行控制，支持两参数格式: "<motorId> <value>"，例如 "0 1.2"
        if (line.empty()) {
            // 空行默认: 控制电机0，值0.2
            custom.oneLegMotion(0, 0.2);
        } else {
            int motorId = 0;
            double val = 0.0;
            double t = 0.0;
            int n = sscanf(line.c_str(), "%d %lf", &motorId, &val);
            if (n == 2) {
            // 两个参数：调用 oneLegMotion(int, double)
            custom.oneLegMotion(motorId, val);
            } else if (sscanf(line.c_str(), "%lf", &t) == 1) {
            // 一个参数：视为时间，调用 oneLegMotion(double, string)
            custom.oneLegMotion(t, std::string());
            } else {
            std::cerr << "Invalid input. Use: <motorId> <value>  or  <time>" << std::endl;
            }
        }
    }

    return 0;
}
