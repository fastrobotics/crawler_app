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
    TankDriveNodeProcess() : mode(Mode::SIMPLE_ARCADE) {
    }
    ~TankDriveNodeProcess();
    // Constants
    /*! \brief Default Output value for Left Drive */
    static constexpr uint16_t LEFTDRIVE_DEFAULT = 1500;

    /*! \brief Default Output value for Right Drive */
    static constexpr uint16_t RIGHTDRIVE_DEFAULT = 1500;

    // Enums

    enum class Mode {
        UNKNOWN = 0,       /*!< Uninitialized value. */
        SIMPLE_ARCADE = 1, /*!< Simple Arcade.  Will have issues at extreme ranges of controls due
                              to output capping. */
        END_OF_LIST = 2    /*!< Last item of list. Used for Range Checks. */
    };

    // Structs
    struct TankDriveNodeProcessContainer {
        TankDriveNodeProcessContainer() {
            left_drive.data = TankDriveNodeProcess::LEFTDRIVE_DEFAULT;
            right_drive.data = TankDriveNodeProcess::RIGHTDRIVE_DEFAULT;
        }
        std_msgs::UInt16 left_drive;
        std_msgs::UInt16 right_drive;
    };

    // Initialization Functions

    eros::eros_diagnostic::Diagnostic finish_initialization();
    void reset();

    // Update Functions
    eros::eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);

    // Attribute Functions
    TankDriveNodeProcessContainer get_drive_command() {
        return drive_command;
    }
    Mode get_mode() {
        return mode;
    }
    void update_armedstate(eros::ArmDisarm::State armed_state) {
        armed_state_ = armed_state;
    }

    // Utility Functions

    // Support Functions
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();

    // Message Functions
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(eros::command msg);
    TankDriveNodeProcessContainer new_cmd_vel(geometry_msgs::Twist cmd_vel_perc);

    // Destructors
    void cleanup() {
        base_cleanup();
        return;
    }

    // Printing Functions
    std::string pretty() override;

   private:
    TankDriveNodeProcessContainer drive_command;
    Mode mode;
    eros::ArmDisarm::State armed_state_;
};
}  // namespace crawler_app
