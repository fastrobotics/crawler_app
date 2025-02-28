#include "TankDriveNodeProcess.h"
namespace crawler_app {
TankDriveNodeProcess::~TankDriveNodeProcess() {
}
eros::eros_diagnostic::Diagnostic TankDriveNodeProcess::finish_initialization() {
    eros::eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    return diag;
}
void TankDriveNodeProcess::reset() {
}
eros::eros_diagnostic::Diagnostic TankDriveNodeProcess::update(double t_dt, double t_ros_time) {
    eros::eros_diagnostic::Diagnostic diag = base_update(t_dt, t_ros_time);
    return diag;
}
std::vector<eros::eros_diagnostic::Diagnostic> TankDriveNodeProcess::new_commandmsg(
    eros::command msg) {
    (void)msg;
    std::vector<eros::eros_diagnostic::Diagnostic> diag_list;
    logger->log_warn("No Command Messages Supported at this time.");
    return diag_list;
}
std::vector<eros::eros_diagnostic::Diagnostic> TankDriveNodeProcess::check_programvariables() {
    std::vector<eros::eros_diagnostic::Diagnostic> diag_list;
    logger->log_warn("No Program Variables Checked.");
    return diag_list;
}
std::string TankDriveNodeProcess::pretty() {
    std::string str = "Node State: " + eros::Node::NodeStateString(get_nodestate());
    return str;
}
TankDriveNodeProcess::TankDriveNodeProcessContainer TankDriveNodeProcess::new_cmd_vel(
    geometry_msgs::Twist cmd_vel_perc) {
    TankDriveNodeProcessContainer output;
    // Simple Convert from Arcade to Tank Drive
    if (mode == Mode::SIMPLE_ARCADE) {
        // Normalize Input to [-1,1]
        double normalized_forward = cmd_vel_perc.linear.x / 100.0;
        double normalized_rotate = cmd_vel_perc.angular.z / 100.0;

        // Invert Rotate due to Right Hand Rule Conventions
        double inverted_rotate = -1.0 * normalized_rotate;

        // Throttle/Steer Mixing
        double left_mixed = normalized_forward + inverted_rotate;
        double right_mixed = normalized_forward - inverted_rotate;

        // Invert Right Channel
        double right_inverted = -1.0 * right_mixed;

        // Scale back to Output range [1000,2000]
        double m = 500.0;
        double b = 1500.0;
        double left_scaled = left_mixed * m + b;
        double right_scaled = right_inverted * m + b;

        // Clip to Min/Max
        double MIN = 1000.0;
        double MAX = 2000.0;
        double left_clipped = left_scaled;
        if (left_clipped > MAX) {
            left_clipped = MAX;
        }
        if (left_clipped < MIN) {
            left_clipped = MIN;
        }

        double right_clipped = right_scaled;
        if (right_clipped > MAX) {
            right_clipped = MAX;
        }
        if (right_clipped < MIN) {
            right_clipped = MIN;
        }

        output.left_drive.data = (uint16_t)(left_clipped);
        output.right_drive.data = (uint16_t)(right_clipped);
        update_diagnostic(eros::eros_diagnostic::DiagnosticType::REMOTE_CONTROL,
                          eros::Level::Type::INFO,
                          eros::eros_diagnostic::Message::NOERROR,
                          "Output Updated");
        drive_command = output;
    }
    else {
        logger->log_warn("Mode: " + std::to_string((uint8_t)mode) + " Not Supported!");
    }
    return output;
}
}  // namespace crawler_app