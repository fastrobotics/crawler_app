/*! \file TankDriveNodeProcess.h
 */
#pragma once
#include <eros/BaseNodeProcess.h>
#include <eros_diagnostic/Diagnostic.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16.h>
/**
 * @brief crawler_app Namespace
 *
 */
namespace crawler_app {
/*! \class TankDriveNodeProcess TankDriveNodeProcess.h "TankDriveNodeProcess.h"
 *  \brief */
class TankDriveNodeProcess : public eros::BaseNodeProcess
{
   public:
    struct TankDriveNodeProcessContainer {
        TankDriveNodeProcessContainer() {
            left_drive.data = 1500;
            right_drive.data = 1500;
        }
        std_msgs::UInt16 left_drive;
        std_msgs::UInt16 right_drive;
    };
    enum class Mode { UNKNOWN = 0, SIMPLE_ARCADE = 1, END_OF_LIST = 2 };
    TankDriveNodeProcess() : mode(Mode::SIMPLE_ARCADE) {
    }
    ~TankDriveNodeProcess();
    eros::eros_diagnostic::Diagnostic finish_initialization();
    void reset();
    eros::eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(eros::command msg);
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();
    void cleanup() {
        base_cleanup();
        return;
    }
    std::string pretty() override;
    TankDriveNodeProcessContainer new_cmd_vel(geometry_msgs::Twist cmd_vel_perc);
    TankDriveNodeProcessContainer get_drive_command() {
        return drive_command;
    }
    Mode get_mode() {
        return mode;
    }

   private:
    TankDriveNodeProcessContainer drive_command;
    Mode mode;
};
}  // namespace crawler_app