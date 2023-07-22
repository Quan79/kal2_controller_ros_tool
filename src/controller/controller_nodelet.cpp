#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "controller.hpp"

namespace kal_controller_ros_tool {

class ControllerNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<ControllerNode>(getPrivateNodeHandle());
    }
    std::unique_ptr<ControllerNode> impl_;
};
} // namespace kal_controller_ros_tool

PLUGINLIB_EXPORT_CLASS(kal_controller_ros_tool::ControllerNodelet, nodelet::Nodelet);
