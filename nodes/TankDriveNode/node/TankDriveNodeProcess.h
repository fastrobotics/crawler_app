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

    // Enums

    /**
     * @brief Tank Drive Mode
     *
     */
    enum class Mode {
        UNKNOWN = 0,       /*!< Uninitialized value. */
        SIMPLE_ARCADE = 1, /*!< Simple Arcade.  Will have issues at extreme ranges of controls due
                              to output capping. */
        END_OF_LIST = 2    /*!< Last item of list. Used for Range Checks. */
    };

    // Structs
    /**
     * @brief Standard output container for Process
     *
     */
    struct TankDriveNodeProcessContainer {
        TankDriveNodeProcessContainer() {
            left_drive.data = 1500;
            right_drive.data = 1500;
        }
        TankDriveNodeProcessContainer(uint16_t left_drive_cmd, uint16_t right_drive_cmd) {
            left_drive.data = left_drive_cmd;
            right_drive.data = right_drive_cmd;
        }
        std_msgs::UInt16 left_drive;
        std_msgs::UInt16 right_drive;
    };

    /**
     * @brief Config Container for Drive Channel
     *
     */
    struct DriveChannelConfig {
        DriveChannelConfig() : min_value(1000), neutral_value(1500), max_value(2000) {
        }
        uint16_t min_value; /*!< Minimum Output value for Output.  Note that if this value is higher
                               than max_value, output is inverted. */
        uint16_t neutral_value; /*!< Neutral/Default Value output.  Will be set when robot is
                                   disarmed. */
        uint16_t max_value; /*!< Maximum Output value for Outpu.  Note that if this value is lower
                               than min_value, output is inverted. */
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
    void set_left_drive_config(DriveChannelConfig config) {
        left_drive_config = config;
    }
    void set_right_drive_config(DriveChannelConfig config) {
        right_drive_config = config;
    }
    DriveChannelConfig get_left_drive_config() {
        return left_drive_config;
    }
    DriveChannelConfig get_right_drive_config() {
        return right_drive_config;
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
    uint16_t clip(uint16_t value, uint16_t min_value, uint16_t max_value);
    DriveChannelConfig left_drive_config;
    DriveChannelConfig right_drive_config;
    TankDriveNodeProcessContainer drive_command;
    Mode mode;
    eros::ArmDisarm::State armed_state_;
};
}  // namespace crawler_app
