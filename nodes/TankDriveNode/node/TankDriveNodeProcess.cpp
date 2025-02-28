#include "TankDriveNodeProcess.h"
namespace crawler_app {
TankDriveNodeProcess::TankDriveNodeProcess() {
}
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
    double forward_cmd = cmd_vel_perc.linear.x / 100.0;
    double angular_cmd = cmd_vel_perc.angular.z / 100.0;
    double left = forward_cmd + angular_cmd;
    double right = forward_cmd - angular_cmd;

    output.left_drive.data = (uint16_t)(left * 1000.0 + 1000);
    output.right_drive.data = (uint16_t)(right * 1000.0 + 1000);
    drive_command = output;
    return output;
}
}  // namespace crawler_app