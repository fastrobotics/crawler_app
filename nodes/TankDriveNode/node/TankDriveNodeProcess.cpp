#include "TankDriveNodeProcess.h"
namespace crawler_app {
TankDriveNodeProcess::~TankDriveNodeProcess() {
    /*
    TankDriveNodeProcessContainer output(left_drive_config.neutral_value,
                                         right_drive_config.neutral_value);
    drive_command = output;
    */
}
eros::eros_diagnostic::Diagnostic TankDriveNodeProcess::finish_initialization() {
    eros::eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    return diag;
}
void TankDriveNodeProcess::reset() {
}
eros::eros_diagnostic::Diagnostic TankDriveNodeProcess::update(double t_dt, double t_ros_time) {
    eros::eros_diagnostic::Diagnostic diag = base_update(t_dt, t_ros_time);
    if (armed_state_.state == eros::ArmDisarm::Type::ARMED) {
        // Do Nothing
    }
    else {
        TankDriveNodeProcessContainer output(left_drive_config.neutral_value,
                                             right_drive_config.neutral_value);
        drive_command = output;
    }
    return diag;
}
std::vector<eros::eros_diagnostic::Diagnostic> TankDriveNodeProcess::new_commandmsg(
    eros::command msg) {
    std::vector<eros::eros_diagnostic::Diagnostic> diag_list = base_new_commandmsg(msg);
    if (diag_list.size() == 0) {
        // No currently supported commands.
    }
    else {
        for (auto diag : diag_list) {
            if (diag.level >= eros::Level::Type::INFO) {
                diagnostic_manager.update_diagnostic(diag);
            }
        }
    }
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
    TankDriveNodeProcessContainer output(left_drive_config.neutral_value,
                                         right_drive_config.neutral_value);
    if (armed_state_.state != eros::ArmDisarm::Type::ARMED) {
        return output;
    }

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

        // Scale back to Output range
        double m_left = (left_drive_config.max_value - left_drive_config.min_value) / (2.0);
        double m_right = (right_drive_config.max_value - right_drive_config.min_value) / (2.0);

        double b_left = left_drive_config.neutral_value - (m_left * 0.0);
        double b_right = right_drive_config.neutral_value - (m_right * 0.0);

        double left_scaled = left_mixed * m_left + b_left;
        double right_scaled = right_inverted * m_right + b_right;
        // Clip to Min/Max
        uint16_t left_clipped = (uint16_t)left_scaled;
        if (left_drive_config.max_value > left_drive_config.min_value) {
            left_clipped =
                clip(left_clipped, left_drive_config.min_value, left_drive_config.max_value);
        }
        else {
            left_clipped =
                clip(left_clipped, left_drive_config.max_value, left_drive_config.min_value);
        }

        uint16_t right_clipped = (uint16_t)right_scaled;
        if (right_drive_config.max_value > right_drive_config.min_value) {
            right_clipped =
                clip(right_clipped, right_drive_config.min_value, right_drive_config.max_value);
        }
        else {
            right_clipped =
                clip(right_clipped, right_drive_config.max_value, right_drive_config.min_value);
        }

        output.left_drive.data = left_clipped;
        output.right_drive.data = right_clipped;
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
uint16_t TankDriveNodeProcess::clip(uint16_t value, uint16_t min_value, uint16_t max_value) {
    uint16_t output = value;
    if (value > max_value) {
        output = max_value;
    }
    if (value < min_value) {
        output = min_value;
    }
    return output;
}
}  // namespace crawler_app