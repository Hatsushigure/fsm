#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <cstdint>
#include <filesystem>
#include <lcm/lcm-cpp.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <string>
#include <string_view>
#include <toml/toml.hpp>
#include <vector>
#include "robot_control_cmd_lcmt.hpp"

/// @brief Find all `.toml` files in given directory
/// @return Relative paths of `.toml` files
std::vector<std::string> findTomlFilesInFolder(std::string_view folderPath)
{
    namespace stdfs = std::filesystem;
    std::vector<std::string> tomlFiles;
    for (
        const auto& entry :
        stdfs::recursive_directory_iterator(folderPath)
        )
    {
        if (entry.is_regular_file() && entry.path().extension() == ".toml")
            tomlFiles.emplace_back(entry.path().string());
    }
    int i = 0;
    for (const auto& filePath : tomlFiles)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("findTomlFilesInFolder"),
            "Found toml file %d: '%s'",
            i,
            filePath.c_str()
        );
        i++;
    }
    return tomlFiles;
}

/// @brief Get array of tables
///
/// ps. Why? Toml already supports this
std::vector<robot_control_cmd_lcmt> getTomlArray(
    const toml::value& tomlData,
    std::size_t arrayLen,
    std::string_view prefix
)
{
    std::vector<robot_control_cmd_lcmt> result; // All messages
    robot_control_cmd_lcmt robotCtrlCmd;        // Temp msg used in loop
    std::stringstream ss;
    for (std::size_t i = 0; i < arrayLen; i++)
    {
        ss.clear();
        ss << prefix << i;
        std::string currentItemName;
        ss >> currentItemName;
        ss.str("");
        RCLCPP_INFO(
            rclcpp::get_logger("getTomlArray"),
            "Current item name: %s",
            currentItemName.c_str()
        );
        const auto& item = toml::find(tomlData, currentItemName);
        robotCtrlCmd.mode = toml::find<std::int8_t>(item, "mode");
        robotCtrlCmd.gait_id = toml::find<std::int8_t>(item, "gait_id");
        robotCtrlCmd.contact = toml::find<std::int8_t>(item, "contact");
        robotCtrlCmd.life_count = toml::find<std::int8_t>(item, "life_count");
        robotCtrlCmd.duration = toml::find<std::int32_t>(item, "duration");
        robotCtrlCmd.value = toml::find<std::int32_t>(item, "value");
        auto velArr = toml::find(item, "vel_des");
        auto posArr = toml::find(item, "pos_des");
        auto rpyArr = toml::find(item, "rpy_des");
        auto accArr = toml::find(item, "acc_des");
        auto ctrlPointArr = toml::find(item, "ctrl_point");
        auto footPoseArr = toml::find(item, "foot_pose");
        auto stepHeightArr = toml::find(item, "step_height");
        for (int j = 0; j < 3; j++)
        {

            robotCtrlCmd.vel_des[j] = toml::find<float>(velArr, j);
            robotCtrlCmd.rpy_des[j] = toml::find<float>(rpyArr, j);
            robotCtrlCmd.pos_des[j] = toml::find<float>(posArr, j);
            robotCtrlCmd.acc_des[j] = toml::find<float>(accArr, j);
            robotCtrlCmd.acc_des[j + 3] = toml::find<float>(accArr, j + 3);
            robotCtrlCmd.ctrl_point[j] = toml::find<float>(ctrlPointArr, j);
            robotCtrlCmd.foot_pose[j] = toml::find<float>(footPoseArr, j);
            robotCtrlCmd.foot_pose[j + 3] = toml::find<float>(footPoseArr, j + 3);
            if (j < 2)
                robotCtrlCmd.step_height[j] = toml::find<float>(stepHeightArr, j);
        }
        result.push_back(robotCtrlCmd);
    }
    return result;
}

/// @brief Publish dog move in a more convenient way
/// @param vel_x [in] X velocity
/// @param vel_y [in] Y velocity
/// @param vel_yaw [in] Yaw velocity
/// @param step_height [in] Step height
/// @param lifeCount [in][out] Heartbeat index
/// @param robotControlCmd [in] Raw control command
/// @param lcm [in] the lcm object used to publish messages
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
    lifeCount = (lifeCount + 1) % 256;
    robotControlCmd.life_count = lifeCount;
    robotControlCmd.step_height[0] = step_height;
    robotControlCmd.step_height[1] = step_height;
    lcm->publish("robot_control_cmd", &robotControlCmd);
}

/// @brief Get whether current status is near target
///
/// ps. I cannot understand what you are talking
inline bool isNearTarget(double dist) { return dist < 0.2; }

#endif  // _UTILS_HPP_