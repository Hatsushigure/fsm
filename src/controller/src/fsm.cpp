#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <string_view>
#include <vector>
#include "pid.hpp"
#include "self_interface/msg/lcmt_position.hpp"
#include "utils.hpp"

#include <stdio.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <filesystem>
#include "robot_control_cmd_lcmt.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using self_interface::msg::LCMTPosition;

double vel_common, vel_right, vel_left, vel_x, test;

//这个打算用于存储那些目标点的坐标，之后根据目标点的坐标用pid修正
std::array<std::array<double, 3>, 50> targetPoints = {
    std::array<double, 3> {0, 0, -1.57}, // 起点0
    {0, 0, 0}, // 起点(转弯)1
    {1.9, 0, 0}, // 沙地2
    {5.2, 0, 0}, // 石子路3
    {8.6, 0, 0}, // 斜坡4
    {8.6, 0, 1.57}, // 转弯5
    {8.61, 3.33, 1.57}, // 绕障6
    {8.61, 3.33, 0}, // 绕障7
    {6.75, 8.55, 3.14}, // 高台阶8
    {4.55, 8.55, 3.14}, // 独木桥9
    {0.3, 8.55, 3.14}, // 矮台阶+石板10
    {0.3, 8.55, -1.57}, // 转弯11
    {0.3, 6.3, -1.57}, // 幕布12    幕布12-18是幕布区域的所有目标点
    {-0.4, 6.3, -1.57},
    {-0.4, 5.3, -1.57},
    {0.3, 5.3, -1.57},
    {0.3, 3.3, -1.57},
    {-0.4, 3.3, -1.57},
    {-0.4, 2.3, -1.57}, // 幕布18
    {9.20, 3.77, 0.65}, // 绕障19   绕障6-7, 19-26是绕障区域所有目标点
    {9.20, 3.77, 1.13}, // 绕障20
    {9.38, 4.16, 1.13}, // 绕障21
    {9.38, 4.16, 1.58}, // 绕障22
    {9.38, 4.5, 1.58},
    {9.38, 4.5, 2.03},
    {8.9, 5.1, 2.15},
    {8.61, 3.45, 0}, // 绕障26
    {8.6, 8.6, 1.57}, // 下台阶
    {8.6, 8.6, 3.14}, // 转弯28
    {8.64, 5.15, 2.53}, // 出绕障29
    {8.64, 5.15, 1.57}, // 转弯（面向台阶）30
    {8.55, 6.67, 1.57},
};

//创建一个枚举变量用于保存状态机的状态
enum FsmState {
    INIT, // 恢复站立
    MOVE1, // 到起点0
    TURN1, // 转弯1
    MOVE2, // 沙地
    MOVE3, // 石子路
    MOVE4, // 上下斜坡
    TURN2, // 转弯2
    MOVE5, // 减速带
    MOVE6, // 站立（循环）一个中间态，用于测试
    MOVE7, // 独木桥
    MOVE8, // 矮台阶+石板
    TURN3, // 转弯4
    MOVE9, // 使用rgb通过幕布，暂时未使用
    MOVE10, // 幕布，MOVE10-MOVE16为通过幕布区域，MOVE17为返回起点
    MOVE11,
    MOVE12,
    MOVE13,
    MOVE14,
    MOVE15,
    MOVE16,
    MOVE17,
    MOVE18, // 绕障（使用pid），MOVE18-MOVE25为使用pid通过绕障区域，效果不好
    MOVE19,
    MOVE20,
    MOVE21,
    MOVE22,
    MOVE23,
    MOVE24,
    MOVE25,
    MOVE26, // 绕障（使用雷达），MOVE26-MOVE28为使用雷达通过绕障区域，效果好
    MOVE27,
    MOVE28,
    MOVE29, // 高台阶
    MOVE30, // （下台阶，下台阶没实现）到转弯点
    TURN4, // 转弯3
    MOVE31, // 出绕障
    TURN5, // 转弯（面向台阶）
    MOVE32, // 转弯后调整
    MOVE33, // 上台阶
    MOVE34,
    MOVE35,
} fsmState;

static bool move33_msg_send = false;

class fsm : public rclcpp::Node
{
private:
    PID pid_y, pid_yaw; //pid的修正
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Subscription<LCMTPosition>::SharedPtr m_dogPosSub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_dogHeartBeatSub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_rgbSub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_lidarSub;
    double gazebo_position_x = 0;
    double gazebo_position_y = 0;
    double gazebo_position_yaw;
    bool m_nearTarget = false; //当前的状态，是否到达目标点附近
    std::shared_ptr<lcm::LCM> m_lcm = nullptr;
    std::int32_t m_lifeCount = 0;//狗的生命值
    std::vector<robot_control_cmd_lcmt> m_mainControlCmdList;
    std::vector<robot_control_cmd_lcmt> m_gaotaiControlCmdList;
    bool rgb_ok = false; // 是否使用rgb相机
    bool rgb_red = false; // 当前rgb相机中是否为红色
    bool lidar_ok; // 是否使用雷达
    bool pass_next = true; // 是否穿过当前幕布
    double m_lidarMidDist = 0;//雷达数据
    double m_lidarStartDist = 0;
    double m_lidarEndDist = 0;
    std::int32_t m_actionNum;
public:
    //构造函数
    fsm(
        std::string_view name,
        std::shared_ptr<lcm::LCM> lcm,
        const std::vector<robot_control_cmd_lcmt>& mainControlCmdList,
        const std::vector<robot_control_cmd_lcmt>& gaotaiControlCmdList,
        std::int32_t actionNum
    ) :
        Node(name.data()),
        m_lcm(lcm),
        m_mainControlCmdList(mainControlCmdList),
        m_gaotaiControlCmdList(gaotaiControlCmdList)
    {
        using namespace std::chrono_literals;
        using namespace std::placeholders;

        fsmState = FsmState::INIT;
        m_actionNum = actionNum;
        // 设置x，y和yaw方向上的kp kd ki
        pid_y.setParameters(0.04, 0.01, 0.03);
        pid_yaw.setParameters(0.03 * 3, 0.01 * 3, 0.02 * 3);

        rclcpp::QoS sensorQosProfile(rclcpp::KeepLast(10)); //配置QoS
        sensorQosProfile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        // 创建定时器，1000ms为周期，定时发布
        m_timer = this->create_wall_timer(10ms, bind(&fsm::timerCallback, this));
        //创建位置订阅者
        m_dogPosSub = this->create_subscription<LCMTPosition>(
            "gazebo_position",
            10,
            std::bind(&fsm::posCallback, this, _1)
        );
        //狗的heartbeat订阅
        m_dogHeartBeatSub = this->create_subscription<std_msgs::msg::Float64>(
            "heart",
            10,
            std::bind(&fsm::heartCallback, this, _1)
        );
        //rgb订阅者
        m_rgbSub = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgb_camera/image_raw",
            sensorQosProfile,
            std::bind(&fsm::rgbCallback, this, _1)
        );
        //lidar订阅者
        m_lidarSub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            sensorQosProfile,
            std::bind(&fsm::lidarCallback, this, _1)
        );
    }

private:
    void changeFsmState(FsmState newState)
    {
        using namespace std::string_view_literals;
        static std::array<std::string_view, 50> state_info = {
            "INIT", "MOVE1", "TURN1", "MOVE2", "MOVE3", "MOVE4", "TURN2", "MOVE5",
            "MOVE6", "MOVE7", "MOVE8", "TURN3", "MOVE9", "MOVE10", "MOVE11", "MOVE12",
            "MOVE13", "MOVE14", "MOVE15", "MOVE16", "MOVE17", "MOVE18", "MOVE19", "MOVE20",
            "MOVE21", "MOVE22", "MOVE23", "MOVE24", "MOVE25", "MOVE26", "MOVE27", "MOVE28",
            "MOVE29", "MOVE30", "TURN4", "MOVE31", "TURN5", "MOVE32", "MOVE33", "MOVE34",
            "MOVE35"};//这个里面的数组长度待定
        pid_y.reset();
        pid_yaw.reset();
        m_nearTarget = false;
        int lastState = int(fsmState);//存放之前的状态
        fsmState = newState;
        RCLCPP_INFO(
            get_logger(),
            "从%s切换到%s",
            state_info[int(lastState)].data(),
            state_info[int(newState)].data()
        );
    }
    void timerCallback() {
        double error_x = 0;
        double error_y = 0;
        double error_yaw = 0;
        robot_control_cmd_lcmt currentRobotCtrlCmd; // 用于存放单个消息        
        robot_control_cmd_lcmt step;
        switch (fsmState)
        {
        case INIT:
            currentRobotCtrlCmd = m_mainControlCmdList[0];
            pubDogMove(
                0,
                0,
                0,
                0,
                m_lifeCount,
                currentRobotCtrlCmd,
                m_lcm
            );
            // if (gazebo_position_y > 0)
            //     fsmState = (enum FsmState)m_actionNum;  //修改此处从不同起始位置开始
            changeFsmState(FsmState::MOVE1);
            break;
        case MOVE1: // 从起点走到第一个转弯处
        {
            error_x = targetPoints[0][1] - gazebo_position_y;
            error_y = targetPoints[0][0] - gazebo_position_x;
            error_yaw = targetPoints[0][2] - gazebo_position_yaw;
            auto yFix = pid_y.calculate(error_y);
            auto yawFix = pid_yaw.calculate(error_yaw);
            auto xVel = 0.3;
            currentRobotCtrlCmd = m_mainControlCmdList[1];
            if (m_nearTarget)
                xVel = 0;
            pubDogMove(
                xVel,
                yFix,
                yawFix,
                0.07,
                m_lifeCount,
                currentRobotCtrlCmd,
                m_lcm
            );
            // 计算目标位置与当前位置的直线距离
            auto dist = std::sqrt(std::pow(error_x, 2) + std::pow(error_y, 2));
            m_nearTarget = isNearTarget(dist);//判断当前是否到目标点附近，如果到目标点附近state设置为false修正yaw和y
            if (dist < 0.2 && std::abs(error_yaw) < 0.1)
                changeFsmState(TURN1);
            break;
        }
        case TURN1: // 沙地前转弯
        {
            error_yaw = targetPoints[1][2] - gazebo_position_yaw;
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            currentRobotCtrlCmd = m_mainControlCmdList[1];
            pubDogMove(
                0,
                0,
                yaw_cmd * 2,
                0.07,
                m_lifeCount,
                currentRobotCtrlCmd,
                m_lcm
            );
            if (std::abs(error_yaw) < 0.1)
                changeFsmState(MOVE2);
            break;
        }
        case MOVE2:  // 通过沙地
        {
            error_x = targetPoints[2][1] - gazebo_position_x;
            error_y = targetPoints[2][1] - gazebo_position_y;
            error_yaw = targetPoints[2][2] - gazebo_position_yaw;
            auto yFix = pid_y.calculate(error_y);
            auto yawFix = pid_yaw.calculate(error_yaw);
            auto xVel = 0.2;
            currentRobotCtrlCmd = m_mainControlCmdList[1];
            if (m_nearTarget)
                xVel = 0;
            pubDogMove(
                xVel,
                yFix,
                yawFix,
                0.07,
                m_lifeCount,
                currentRobotCtrlCmd,
                m_lcm
            );
            auto dist = sqrt(pow(error_x, 2) + pow(error_y, 2));
            m_nearTarget = isNearTarget(dist);
            if (dist < 0.2 && error_yaw < 0.1)
                changeFsmState(MOVE3);
            break;
        }
        case MOVE3: // 通过石子路+上斜坡
        {
            currentRobotCtrlCmd = m_mainControlCmdList[2];
            pubDogMove(
                0.15,
                0,
                0,
                0.15,
                m_lifeCount,
                currentRobotCtrlCmd,
                m_lcm
            );
            if (gazebo_position_x > 5.2)
                changeFsmState(MOVE4);
            break;
        }
        case MOVE4: { // 下斜坡
            error_x = targetPoints[4][1] - gazebo_position_y;
            error_y = targetPoints[4][0] - gazebo_position_x;
            error_yaw = targetPoints[4][2] - gazebo_position_yaw;
            auto y_cmd = pid_y.calculate(error_x);
            auto yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget)
            {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0.4, y_cmd, yaw_cmd, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, y_cmd, yaw_cmd, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            std::double_t dist = sqrt(pow(error_x, 2) + pow(error_y, 2));
            m_nearTarget = isNearTarget(dist);
            if (dist < 0.2 && error_yaw < 0.1) {
                changeFsmState(TURN2);
            }
            break;
        }
        case TURN2: { // 
            error_yaw = targetPoints[5][2] - gazebo_position_yaw;
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            currentRobotCtrlCmd = m_mainControlCmdList[1];
            pubDogMove(0, 0, yaw_cmd * 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            if (abs(error_yaw) < 0.1) {
                changeFsmState(MOVE5);
            }
            break;
        }
        case MOVE5: { // 通过减速带
            error_x = targetPoints[6][1] - gazebo_position_y;
            error_y = targetPoints[6][0] - gazebo_position_x;
            error_yaw = targetPoints[6][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0.4, -y_cmd, yaw_cmd, 0.12, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, -y_cmd, yaw_cmd, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            std::double_t dist = sqrt(pow(error_x, 2) + pow(error_y, 2));
            if (dist < 0.2 || gazebo_position_y > 3.0)
                m_nearTarget = true;
            else
                m_nearTarget = false;
            if (dist < 0.2 && error_yaw < 0.1) {
                changeFsmState(MOVE26);
            }
            break;
        }
        case MOVE6: { // 一个中间态，让狗保持站立
            currentRobotCtrlCmd = m_mainControlCmdList[0];
            pubDogMove(0, 0, 0, 0, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            fsmState = MOVE6;
            break;
        }
        case MOVE7: { // 通过独木桥
            error_x = targetPoints[9][1] - gazebo_position_y;
            error_y = targetPoints[9][0] - gazebo_position_x;
            if (gazebo_position_yaw < 0) {
                gazebo_position_yaw += 6.28;
            }
            error_yaw = targetPoints[9][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_x);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0.3, -y_cmd, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, -y_cmd, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            std::double_t dist = sqrt(pow(error_x, 2) + pow(error_y, 2));
            m_nearTarget = isNearTarget(dist);
            if (dist < 0.2 && error_yaw < 0.1) {
                changeFsmState(MOVE8);
            }
            break;
        }
        case MOVE8: { // 下矮台阶+石板路
            error_x = targetPoints[10][1] - gazebo_position_y;
            error_y = targetPoints[10][0] - gazebo_position_x;
            if (gazebo_position_yaw < 0) {
                gazebo_position_yaw += 6.28;
            }
            error_yaw = targetPoints[10][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_x);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[3];
                pubDogMove(0.3, -y_cmd, yaw_cmd, 0.08, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, -y_cmd, yaw_cmd, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            std::double_t dist = sqrt(pow(error_x, 2) + pow(error_y, 2));
            m_nearTarget = isNearTarget(dist);
            if (dist < 0.2) {
                changeFsmState(TURN3);
            }
            break;
        }
        case TURN3: { // 
            if (gazebo_position_yaw > 0) {
                gazebo_position_yaw -= 6.28;
            }
            error_yaw = targetPoints[11][2] - gazebo_position_yaw;
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);

            currentRobotCtrlCmd = m_mainControlCmdList[1];
            pubDogMove(0, 0, yaw_cmd * 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);

            if (abs(error_yaw) < 0.1) {
                changeFsmState(MOVE10);
                // lidar_ok = true;
                // rgb_ok = true;
            }
            break;
        }
        case MOVE9: { // 视觉识别幕布，暂时不用
            if (rgb_red && pass_next) {//有红色并且是已经通过了后面那个，说明要往左边或者右边走了
                //首先我要确保狗是正的，这时在这里就可以用lidar了
                //如果左边大于右边 则说明要往左走，否则往右走，只水平平移，不前进
                if (m_lidarStartDist > m_lidarEndDist && m_lidarEndDist > 0.05) {
                    currentRobotCtrlCmd = m_mainControlCmdList[1];
                    pubDogMove(0, 0.1, 0, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
                }
                else if (m_lidarStartDist <= m_lidarEndDist && m_lidarStartDist > 0.05) {
                    currentRobotCtrlCmd = m_mainControlCmdList[1];
                    pubDogMove(0, -0.1, 0, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
                }
            }
            else if (!rgb_red || !pass_next) {//直行 并且在直行的过程中计算出和前面的距离，当有一定距离后停止
                pass_next = false;//首先设置为false
                //这个时候说明前方是绿色的了，首先直行 这个我觉得是根据lidar的距离用pid来让他通过
                //如果穿过了就设置pass_next为true
                if (6.9 < m_lidarMidDist < 7.1 || 5.9 < m_lidarMidDist < 6.1 ||
                    4.9 < m_lidarMidDist < 5.1 || 3.9 < m_lidarMidDist < 4.1) {
                    pass_next = true;
                }
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0.2, 0, 0, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            break;
        }
        case MOVE10: { // 幕布起点
            error_x = targetPoints[12][1] - gazebo_position_y;
            error_y = targetPoints[12][0] - gazebo_position_x;
            error_yaw = targetPoints[12][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0.3, y_cmd, yaw_cmd, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, y_cmd, yaw_cmd, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            std::double_t dist = sqrt(pow(error_x, 2) + pow(error_y, 2));
            m_nearTarget = isNearTarget(dist);
            if (dist < 0.2 && error_yaw < 0.1) {
                changeFsmState(MOVE11);
            }
            break;
        }
        case MOVE11: {
            error_x = targetPoints[13][1] - gazebo_position_y;
            error_y = targetPoints[13][0] - gazebo_position_x;
            error_yaw = targetPoints[13][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, -0.1, 0, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, y_cmd * 2, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            std::double_t dist = sqrt(pow(error_x, 2) + pow(error_y, 2));
            m_nearTarget = isNearTarget(dist);
            if (dist < 0.2 && error_yaw < 0.1) {
                changeFsmState(MOVE12);
            }
            break;
        }
        case MOVE12: {
            error_x = targetPoints[14][1] - gazebo_position_y;
            error_y = targetPoints[14][0] - gazebo_position_x;
            error_yaw = targetPoints[14][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0.3, y_cmd, yaw_cmd, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, y_cmd, yaw_cmd, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            std::double_t dist = sqrt(pow(error_x, 2) + pow(error_y, 2));
            m_nearTarget = isNearTarget(dist);
            if (dist < 0.2 && error_yaw < 0.1) {
                changeFsmState(MOVE13);
            }
            break;
        }
        case MOVE13: {
            error_x = targetPoints[15][1] - gazebo_position_y;
            error_y = targetPoints[15][0] - gazebo_position_x;
            error_yaw = targetPoints[15][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, 0.1, 0, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, y_cmd * 2, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            std::double_t dist = sqrt(pow(error_x, 2) + pow(error_y, 2));
            m_nearTarget = isNearTarget(dist);
            if (dist < 0.2 && error_yaw < 0.1) {
                changeFsmState(MOVE14);
            }
            break;
        }
        case MOVE14: {
            error_x = targetPoints[16][1] - gazebo_position_y;
            error_y = targetPoints[16][0] - gazebo_position_x;
            error_yaw = targetPoints[16][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0.3, y_cmd, yaw_cmd, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, y_cmd, yaw_cmd, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            std::double_t dist = sqrt(pow(error_x, 2) + pow(error_y, 2));
            m_nearTarget = isNearTarget(dist);
            if (dist < 0.2 && error_yaw < 0.1) {
                changeFsmState(MOVE15);
            }
            break;
        }
        case MOVE15: {
            error_x = targetPoints[17][1] - gazebo_position_y;
            error_y = targetPoints[17][0] - gazebo_position_x;
            error_yaw = targetPoints[17][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, -0.1, 0, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, y_cmd * 2, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            std::double_t dist = sqrt(pow(error_x, 2) + pow(error_y, 2));
            m_nearTarget = isNearTarget(dist);
            if (dist < 0.2 && error_yaw < 0.1) {
                changeFsmState(MOVE16);
            }
            break;
        }
        case MOVE16: {
            error_x = targetPoints[18][1] - gazebo_position_y;
            error_y = targetPoints[18][0] - gazebo_position_x;
            error_yaw = targetPoints[18][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0.3, y_cmd, yaw_cmd, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, y_cmd, yaw_cmd, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            std::double_t dist = sqrt(pow(error_x, 2) + pow(error_y, 2));
            m_nearTarget = isNearTarget(dist);
            if (dist < 0.2 && error_yaw < 0.1) {
                changeFsmState(MOVE17);
            }
            break;
        }
        case MOVE17: { // 通过幕布，返回起点
            error_x = targetPoints[0][1] - gazebo_position_y;
            error_y = targetPoints[0][0] - gazebo_position_x;
            error_yaw = targetPoints[0][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0.3, y_cmd, yaw_cmd, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, y_cmd, yaw_cmd, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            std::double_t dist = sqrt(pow(error_x, 2) + pow(error_y, 2));
            m_nearTarget = isNearTarget(dist);
            if (dist < 0.2 && error_yaw < 0.1) {
                changeFsmState(MOVE6);
            }
            break;
        }
        case MOVE18: { //pid绕障起点
            error_x = targetPoints[7][1] - gazebo_position_y;
            error_y = targetPoints[7][0] - gazebo_position_x;
            error_yaw = targetPoints[7][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            currentRobotCtrlCmd = m_mainControlCmdList[1];
            pubDogMove(0, -y_cmd / 2, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            if (error_yaw > 0) {
                changeFsmState(MOVE6);
            }
            break;
        }
        case MOVE19: {
            error_x = targetPoints[19][1] - gazebo_position_y;
            error_y = targetPoints[19][0] - gazebo_position_x;
            error_yaw = targetPoints[19][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0.1, -y_cmd / 2, yaw_cmd / 3, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, -y_cmd / 2, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            if (gazebo_position_y > targetPoints[19][1] || gazebo_position_x > targetPoints[19][0]) {
                changeFsmState(MOVE20);
            }
            break;
        }
        case MOVE20: {
            error_y = targetPoints[20][0] - gazebo_position_x;
            error_yaw = targetPoints[20][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            currentRobotCtrlCmd = m_mainControlCmdList[1];
            pubDogMove(0, 0, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            if (error_yaw < 0) {
                changeFsmState(MOVE21);
            }
            break;
        }
        case MOVE21: {
            error_x = targetPoints[21][1] - gazebo_position_y;
            error_y = targetPoints[21][0] - gazebo_position_x;
            error_yaw = targetPoints[21][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0.1, -y_cmd / 2, yaw_cmd / 3, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, -y_cmd / 2, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            if (gazebo_position_y > targetPoints[21][1] || gazebo_position_x > targetPoints[21][0]) {
                changeFsmState(MOVE22);
            }
            break;
        }
        case MOVE22: {
            error_y = targetPoints[22][0] - gazebo_position_x;
            error_yaw = targetPoints[22][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            currentRobotCtrlCmd = m_mainControlCmdList[1];
            pubDogMove(0, -y_cmd / 2, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            if (error_yaw < 0) {
                changeFsmState(MOVE23);
            }
            break;
        }
        case MOVE23: {
            error_x = targetPoints[23][1] - gazebo_position_y;
            error_y = targetPoints[23][0] - gazebo_position_x;
            error_yaw = targetPoints[23][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0.1, -y_cmd / 2, yaw_cmd / 3, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, -y_cmd / 2, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            if (gazebo_position_y > targetPoints[23][1] || gazebo_position_x < targetPoints[23][0]) {
                changeFsmState(MOVE24);
            }
            break;
        }
        case MOVE24: {
            error_y = targetPoints[24][0] - gazebo_position_x;
            error_yaw = targetPoints[24][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            currentRobotCtrlCmd = m_mainControlCmdList[1];
            pubDogMove(0, -y_cmd / 2, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            if (error_yaw < 0) {
                changeFsmState(MOVE25);
            }
            break;
        }
        case MOVE25: { // pid绕障结束
            error_x = targetPoints[25][1] - gazebo_position_y;
            error_y = targetPoints[25][0] - gazebo_position_x;
            error_yaw = targetPoints[25][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0.1, 0, yaw_cmd / 3, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, -y_cmd / 2, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            if (gazebo_position_y > targetPoints[25][1] || gazebo_position_x < targetPoints[25][0]) {
                changeFsmState(MOVE6);
            }
            break;
        }
        case MOVE26: { // 雷达绕障起点
            error_yaw = targetPoints[7][2] - gazebo_position_yaw;
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);

            currentRobotCtrlCmd = m_mainControlCmdList[1];
            pubDogMove(0, 0, yaw_cmd * 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);

            if (abs(error_yaw) < 0.1) {
                changeFsmState(MOVE27);
            }
            break;
        }
        case MOVE27: {
            error_x = targetPoints[26][1] - gazebo_position_y;
            error_y = targetPoints[26][0] - gazebo_position_x;
            error_yaw = targetPoints[26][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_x);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                if (error_y < 0) {
                    pubDogMove(0, y_cmd * 3, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
                }
                else {
                    pubDogMove(0.05, y_cmd * 3, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
                }
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, y_cmd / 2, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            if (gazebo_position_y > targetPoints[26][1]) {
                changeFsmState(MOVE28);
                lidar_ok = true;
            }
            break;
        }
        case MOVE28: { // 雷达绕障结束
            currentRobotCtrlCmd = m_mainControlCmdList[1];
            RCLCPP_INFO(this->get_logger(), "vel_x:%lf", vel_x);
            if (m_lidarStartDist < 0.28) {
                // RCLCPP_INFO(this->get_logger(),"m_lidarStartDist:%lf", m_lidarStartDist);
                pubDogMove(vel_x, 0, vel_right, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else if (m_lidarEndDist < 0.3) {
                // RCLCPP_INFO(this->get_logger(),"m_lidarEndDist:%lf", m_lidarEndDist);
                pubDogMove(vel_x, 0, vel_left, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                // RCLCPP_INFO(this->get_logger(),"common");
                pubDogMove(vel_x, 0, vel_common, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            if (gazebo_position_y > 4.88) {
                changeFsmState(MOVE31);
                lidar_ok = false;
            }
            break;
        }
        case MOVE29: { // 上楼梯
            currentRobotCtrlCmd = m_mainControlCmdList[2];
            static bool move29_msg_sent = false;
            if (!move29_msg_sent) {
                pubDogMove(0.15, 0, 0, 0.15, m_lifeCount, currentRobotCtrlCmd, m_lcm);
                move29_msg_sent = true;
            }
            if (gazebo_position_x < 6.75) {
                changeFsmState(MOVE7);
            }
            break;
        }
        case MOVE30: { // 下高台后走到转弯点
            error_x = targetPoints[27][1] - gazebo_position_y;
            error_y = targetPoints[27][0] - gazebo_position_x;
            error_yaw = targetPoints[27][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0.3, -y_cmd, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, -y_cmd, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            std::double_t dist = sqrt(pow(error_x, 2) + pow(error_y, 2));
            m_nearTarget = isNearTarget(dist);
            if (dist < 0.2 && error_yaw < 0.1) {
                changeFsmState(TURN4);
            }
            break;
        }
        case TURN4: { // 
            error_yaw = targetPoints[28][2] - gazebo_position_yaw;
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            currentRobotCtrlCmd = m_mainControlCmdList[1];
            pubDogMove(0, 0, yaw_cmd * 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            if (abs(error_yaw) < 0.1) {
                changeFsmState(MOVE29);
            }
            break;
        }
        case MOVE31: { // 出绕障
            currentRobotCtrlCmd = m_mainControlCmdList[1];
            pubDogMove(0.1, 0, 0, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            if (gazebo_position_y > targetPoints[29][1] || gazebo_position_x < targetPoints[29][0]) {
                changeFsmState(TURN5);
            }
            break;
        }
        case TURN5: { // 转弯面向高台
            error_yaw = targetPoints[30][2] - gazebo_position_yaw;
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            currentRobotCtrlCmd = m_mainControlCmdList[1];
            pubDogMove(0, 0, yaw_cmd * 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            if (abs(error_yaw) < 0.1) {
                changeFsmState(MOVE32);
            }
            break;
        }
        case MOVE32: { // 高台前调整（横向移动）
            error_y = targetPoints[30][0] - gazebo_position_x;
            std::double_t y_cmd = pid_y.calculate(error_y);
            currentRobotCtrlCmd = m_mainControlCmdList[1];
            pubDogMove(0, -y_cmd * 2, 0, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            if (gazebo_position_x < targetPoints[30][0]) {
                changeFsmState(MOVE33);
            }
            break;
        }
        case MOVE33: { // 序列动作上高台
            if (!move33_msg_send) {
                for (int i = 0; i < m_gaotaiControlCmdList.size(); i++) {
                    step = m_gaotaiControlCmdList[i];
                    m_lcm->publish("robot_control_cmd", &step);
                    rclcpp::sleep_for(std::chrono::milliseconds(200));
                }
                move33_msg_send = true;
            }
        }
        case MOVE34: { // 高台顶调位置
            error_x = targetPoints[31][1] - gazebo_position_y;
            error_y = targetPoints[31][0] - gazebo_position_x;
            error_yaw = targetPoints[31][2] - gazebo_position_yaw;
            std::double_t y_cmd = pid_y.calculate(error_y);
            std::double_t yaw_cmd = pid_yaw.calculate(error_yaw);
            if (!m_nearTarget) {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0.1, -y_cmd, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            else {
                currentRobotCtrlCmd = m_mainControlCmdList[1];
                pubDogMove(0, -y_cmd, yaw_cmd / 2, 0.07, m_lifeCount, currentRobotCtrlCmd, m_lcm);
            }
            if (gazebo_position_y > test) {
                m_nearTarget = true;
            }
            else {
                m_nearTarget = false;
            }
            if (gazebo_position_y > test && abs(error_yaw) < 0.1) {
                changeFsmState(MOVE35);
            }
            break;
        }
        case MOVE35: { // 下高台
            currentRobotCtrlCmd = m_mainControlCmdList[4];
            int tmp1 = currentRobotCtrlCmd.mode, tmp2 = currentRobotCtrlCmd.gait_id;
            RCLCPP_INFO(this->get_logger(), "mode:%lf :%lf", tmp1, tmp2);
            m_lcm->publish("robot_control_cmd", &currentRobotCtrlCmd);
            currentRobotCtrlCmd = m_mainControlCmdList[0];
            currentRobotCtrlCmd.duration = 500;
            m_lcm->publish("robot_control_cmd", &currentRobotCtrlCmd);
            currentRobotCtrlCmd = m_mainControlCmdList[4];
            m_lcm->publish("robot_control_cmd", &currentRobotCtrlCmd);
            if (gazebo_position_y > 8.0) {
                changeFsmState(MOVE30);
            }
        }
        }
    }

    void posCallback(const self_interface::msg::LCMTPosition& msg)
    {
        gazebo_position_x = msg.cydog_pose.position.x;
        gazebo_position_y = msg.cydog_pose.position.y;
        tf2::Matrix3x3 quat_data(tf2::Quaternion(msg.facing.x, msg.facing.y,
            msg.facing.z, msg.facing.w));

        double current_roll, current_pitch, current_yaw;
        quat_data.getRPY(current_roll, current_pitch, current_yaw);
        gazebo_position_yaw = current_yaw;
        // RCLCPP_INFO(this->get_logger(),"x:%lf", gazebo_position_x);
        // RCLCPP_INFO(this->get_logger(),"y:%lf", gazebo_position_y);
        // RCLCPP_INFO(this->get_logger(),"yaw:%lf", gazebo_position_yaw);
    }

    void heartCallback(const std_msgs::msg::Float64 msg) {
        if (!m_lcm->good()) {
            RCLCPP_INFO(this->get_logger(), "no Heat beat");
            return;
        }
        else {
            m_mainControlCmdList[0].life_count = m_lifeCount;
            m_lcm->publish("robot_control_cmd", &m_mainControlCmdList[0]);
            // RCLCPP_INFO(this->get_logger(), "Heat beat");
        }
    }

    // 加一个state,只有当开始转弯之后才开始让他处理这个话题
    void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (rgb_ok) {
            // 将ROS图像消息转换为OpenCV图像
            // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            // cv::Mat img1= cv_ptr->image;
            // cv::Mat current_Image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
            cv::Mat current_Image(msg->height, msg->width, CV_8UC3, const_cast<uchar*>(msg->data.data()), msg->step > 0 ? msg->step : msg->width * 3);
            // cv::Mat current_Image = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::imshow("Resized Image", current_Image);
            cv::waitKey(1);
            // 获取图像中间部分
            auto height = current_Image.rows;
            auto width = current_Image.cols;
            auto center_x = width / 2;
            auto center_y = height / 2;
            auto offset = 50;
            cv::Rect center_region(center_x - offset, center_y - offset, offset * 2, offset * 2);
            cv::Mat center_image = current_Image(center_region);
            if (center_image.empty()) {
                RCLCPP_INFO(this->get_logger(), "Error: Could not open the image");
            }
            else {
                // 显示图像
                // cv::imshow("Resized Image", current_Image);
                // cv::waitKey(1);
            }
            // 将图像转换为HSV颜色空间
            cv::Mat hsv_image;
            cv::cvtColor(center_image, hsv_image, cv::COLOR_BGR2HSV);
            // 定义红色和绿色的HSV颜色范围
            cv::Scalar lower_red(0, 120, 70);
            cv::Scalar upper_red(10, 255, 255);
            cv::Scalar lower_green(36, 100, 100);
            cv::Scalar upper_green(86, 255, 255);
            // 创建红色和绿色的掩膜
            cv::Mat red_mask, green_mask;
            cv::inRange(hsv_image, lower_red, upper_red, red_mask);
            cv::inRange(hsv_image, lower_green, upper_green, green_mask);

            // 创建窗口
            cv::namedWindow("Green Mask", cv::WINDOW_NORMAL);

            // 设置窗口大小 (宽度, 高度)
            cv::resizeWindow("Green Mask", 600, 400); // 例如 600x400 像素

            cv::imshow("Green Mask", green_mask);

            cv::namedWindow("Red Mask", cv::WINDOW_NORMAL);

            // 设置窗口大小 (宽度, 高度)
            cv::resizeWindow("Red Mask", 600, 400); // 例如 600x400 像素

            cv::imshow("Red Mask", red_mask);

            // 计算中间区域中红色和绿色像素的数量
            std::int32_t red_count = cv::countNonZero(red_mask);
            std::int32_t green_count = cv::countNonZero(green_mask);

            // RCLCPP_INFO(this->get_logger(), "red_count:%lf", red_count);
            // RCLCPP_INFO(this->get_logger(), "green_count:%lf", green_count);
            // 判断中间区域的颜色
            // 这里应该改为如果有红色的就往左边走
            if (red_count > 200) {
                rgb_red = true;
            }
            else {
                rgb_red = false;
            }
        }
    }

    void lidarCallback(const sensor_msgs::msg::LaserScan& msg) {
        if (lidar_ok) {
            std::int32_t lidarRange = msg.ranges.size();
            std::int32_t lidarMid = lidarRange / 2;
            m_lidarStartDist = msg.ranges[0];
            m_lidarMidDist = msg.ranges[lidarMid];//90度
            m_lidarEndDist = msg.ranges[179];
        }
    }
};

int main(int argc, char** argv)
{
    // 读取toml文件获得动作序列到robot_control_cmds_vector
    // std::string base = "./";
    // std::vector<std::string> action_toml_path = findTomlFilesInFolder(base);
    // int num = 0;
    // std::cout<<"请输入对应toml文件的标号:";
    // std::cin>>num;
    // int toml_length = 0;

    const toml::value data = toml::parse("./src/controller/src/cyberdog2_ctrl.toml");

    const auto& count_set = toml::find(data, "count_set");
    const auto mainCtrlListLength = toml::find<std::int32_t>(count_set, "step_len");
    const auto actionNum = toml::find<std::int32_t>(count_set, "action_num");
    const auto toml_gaotai_len = toml::find<std::int32_t>(count_set, "gaotai_len");

    vel_common = toml::find<std::double_t>(count_set, "vel_common");
    vel_right = toml::find<std::double_t>(count_set, "vel_right");
    vel_left = toml::find<std::double_t>(count_set, "vel_left");
    vel_x = toml::find<std::double_t>(count_set, "vel_x");
    test = toml::find<std::double_t>(count_set, "test");
    const auto prefix = toml::find<std::string>(count_set, "prefix");
    const auto prefix_gaotai = toml::find<std::string>(count_set, "gaotai_prefix");


    std::vector<robot_control_cmd_lcmt> mainCtrlCmdList = getTomlArray(
        data,
        mainCtrlListLength,
        prefix
    );

    // const toml::value data_gaotai = toml::parse("./src/controller/src/platform_v1.toml");
    const toml::value data_gaotai = toml::parse("./src/controller/src/cyberdog2_ctrl.toml");

    std::vector<robot_control_cmd_lcmt> gaotaiCtrlCmdList = getTomlArray(
        data_gaotai,
        toml_gaotai_len,
        prefix_gaotai
    );

    rclcpp::init(argc, argv);
    auto lcm = std::make_shared<lcm::LCM>("udpm://239.255.76.67:7671?ttl=255");
    auto node = std::make_shared<fsm>(
        "fsm",
        lcm,
        mainCtrlCmdList,
        gaotaiCtrlCmdList,
        actionNum
    );
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
