#pragma once

#include <optional>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf2_ros/transform_listener.h>

#include <kal_controller/controller.hpp>
#include <kal_controller/types.hpp>

#include "kal_controller_ros_tool/ControllerInterface.h"
#include <detection_msgs/BoundingBoxes.h>
#include <detection_msgs/BoundingBox.h>

namespace kal_controller_ros_tool {

using namespace kal_controller;

class ControllerNode {
public:
    using Interface = ControllerInterface;

    explicit ControllerNode(const ros::NodeHandle& nhPrivate);

private:
    /**
     * @brief Callback to receive and store a new trajectory.
     *
     * @param pathMsg
     */
    void trajectoryCallback(const nav_msgs::Path::ConstPtr& msg);

    /**
     * @brief Timed callback to compute control commands from current trajectory and vehicle pose
     *
     * @param timerEvent
     */
    void controlLoopCallback(const ros::TimerEvent& timer);

    /**
     * @brief Dynamic reconfigure callback to update parameter values.
     *
     * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window.
     * The parameter "level" is unused here. It is set to the number of changes in the config.
     * At startup, the callback is automatically called with level == std::numeric_limits<uint32_t>::max().
     */
    void reconfigureCallback(const Interface::Config& config, uint32_t /*level*/);

    /**
     * @brief Read current vehicle pose from tf tree
     *
     * @param pose Return parameter. Contains the pose, if found.
     * @return true Pose found successfully
     * @return false Pose not found
     */
    bool currentVehiclePose(Pose& pose) const;

    /**
     * @brief Check that trajectory has been defined. Send warning and stop the car if it hasn't.
     *
     * @return True if trajectory exists, false if it doesn't
     */
    bool checkTrajectoryExists() const;

    /**
     * @brief Check that trajectory isn't older than maximum allowed age. Send warning and stop the car if it hasn't.
     *
     * @return True if trajectory time stamp is valid, false if it is too old
     */
    bool checkTrajectoryAge() const;

    /**
     * @brief Check that trajectory has the minimum required length.
     *
     * @return True if trajectory is long enough, false if it isn't
     */
    bool checkTrajectoryLength() const;

    /**
     * @brief Publish a control command to stop the car.
     *
     */
    void publishStopCommand() const;

    /**
     * @brief Publish debug info.
     *
     */
    void publishDebugInfo(const ControlCommand::DebugInfo& debugInfo, const ros::Time& stamp) const;

    void setControllerParameters();
    void stopVehicle();
    void yoloCallback(const detection_msgs::BoundingBoxes::ConstPtr& msg);
    void startVehicle();

    Controller controller_{};
    std::optional<Trajectory> trajectory_{std::nullopt};
    ros::Time trajectoryStamp_{0};

    ros::Timer controlLoopTimer_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_{tfBuffer_};

    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;
};
} // namespace kal_controller_ros_tool
