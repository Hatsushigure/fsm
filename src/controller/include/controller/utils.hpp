#ifndef __MY_UTILS_H__
#define __MY_UTILS_H__

#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "robot_control_cmd_lcmt.hpp"
#include "toml/toml.hpp"


/// @brief 查找指定目录下所有 toml 文件
/// @return 文件相对路径的列表
std::vector<std::string> findTomlFilesInFolder(std::string_view folderPath) {
    std::vector<std::string> tomlFiles;
    for (
        const auto& entry :
        std::filesystem::recursive_directory_iterator(folderPath)
        )
    {
        if (entry.is_regular_file() && entry.path().extension() == ".toml")
            tomlFiles.emplace_back(entry.path());
    }
    int i = 0;
    for (const auto& filePath : tomlFiles)
    {
        std::cout << i << ": toml文件: " << filePath << std::endl;
        i++;
    }
    return tomlFiles;
}


/// @brief 获取表数组的内容
///
/// ps.我不理解为什么要脱裤子放屁
std::vector<robot_control_cmd_lcmt> getTomlArray(
    const toml::value& tomlData,
    std::int32_t length,
    std::string_view prefix
)
{
    std::vector<robot_control_cmd_lcmt> result; // 用于存放所有的消息
    robot_control_cmd_lcmt robotCtrlCmd;   // 用于存放单个消息
    std::stringstream ss;
    for (auto i = 0; i < length; i++)
    {
        ss.clear();
        ss << prefix << i;
        auto currentItemName = ss.str();
        std::cout << currentItemName << '\n';
        auto& test = toml::find(tomlData, currentItemName);    //这里得到的是该step的所有内容
        robotCtrlCmd.mode = toml::find<std::int8_t>(test, "mode");
        robotCtrlCmd.gait_id = toml::find<std::int8_t>(test, "gait_id");
        robotCtrlCmd.contact = toml::find<std::int8_t>(test, "contact");
        robotCtrlCmd.life_count = toml::find<std::int32_t>(test, "life_count");
        robotCtrlCmd.duration = toml::find<std::int32_t>(test, "duration");
        robotCtrlCmd.value = toml::find<std::int32_t>(test, "value");
        auto vel_get = toml::find(test, "vel_des");
        auto pos_get = toml::find(test, "pos_des");
        auto rpy_get = toml::find(test, "rpy_des");
        auto acc_get = toml::find(test, "acc_des");
        auto ctrl_get = toml::find(test, "ctrl_point");
        auto foot_get = toml::find(test, "foot_pose");
        auto height_get = toml::find(test, "step_height");
        for (int j = 0; j < 3; j++)
        {

            robotCtrlCmd.vel_des[j] = toml::find<float>(vel_get, j);
            robotCtrlCmd.rpy_des[j] = toml::find<float>(rpy_get, j);
            robotCtrlCmd.pos_des[j] = toml::find<float>(pos_get, j);
            robotCtrlCmd.acc_des[j] = toml::find<float>(acc_get, j);
            robotCtrlCmd.acc_des[j + 3] = toml::find<float>(acc_get, j + 3);
            robotCtrlCmd.ctrl_point[j] = toml::find<float>(ctrl_get, j);
            robotCtrlCmd.foot_pose[j] = toml::find<float>(foot_get, j);
            robotCtrlCmd.foot_pose[j + 3] = toml::find<float>(foot_get, j + 3);
            if (j < 2)
                robotCtrlCmd.step_height[j] = toml::find<float>(height_get, j);
        }
        result.push_back(robotCtrlCmd);
    }
    return result;
}

/// @brief 简化发布动作的过程
/// @param vel_x [in] 发布的狗 x 方向速度
/// @param vel_y [in] 发布的狗 y 方向速度
/// @param vel_yaw [in] 发布的狗 yaw 方向速度
/// @param step_height [in] 抬腿高度
/// @param lifeCount [in][out] 生命值
/// @param robotControlCmd [in] 对应的 toml 动作
/// @param lcm [in] 用于发布消息的 lcm 对象
void pubDogMove(
    double vel_x,
    double vel_y,
    double vel_yaw,
    double step_height,
    std::int32_t& lifeCount,
    robot_control_cmd_lcmt robotControlCmd,
    std::shared_ptr<lcm::LCM> lcm
)
{
    robotControlCmd.vel_des[0] = vel_x;
    robotControlCmd.vel_des[1] = vel_y;
    robotControlCmd.vel_des[2] = vel_yaw;
    lifeCount++;
    robotControlCmd.life_count = lifeCount;
    robotControlCmd.step_height[0] = step_height;
    robotControlCmd.step_height[1] = step_height;
    lcm->publish("robot_control_cmd", &robotControlCmd);
}

/// @brief 判断当前状态是否到目标点附近
bool getStatus(double dist)
{
    bool state;
    if (dist < 0.2)
        state = false;
    else
        state = true;
    return state;
}
#endif // __MY_UTILS_H__
