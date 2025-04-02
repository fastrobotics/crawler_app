/*! \file test_TankDriveNodeProcess.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "../TankDriveNodeProcess.h"
namespace crawler_app {
class TankDriveNodeProcessTester : public TankDriveNodeProcess
{
   public:
    TankDriveNodeProcessTester() {
    }
    ~TankDriveNodeProcessTester() {
    }
};
}  // namespace crawler_app
using namespace crawler_app;
TEST(BasicTest, TestOperation) {
    eros::Logger* logger = new eros::Logger("DEBUG", "UnitTestTankDriveNodeProcess");
    TankDriveNodeProcessTester* tester = new TankDriveNodeProcessTester;
    tester->initialize("UnitTestTankDriveNodeProcess",
                       "UnitTestTankDriveNodeProcess",
                       "MyHost",
                       eros::System::MainSystem::SIMROVER,
                       eros::System::SubSystem::ENTIRE_SYSTEM,
                       eros::System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<eros::eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::SOFTWARE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::DATA_STORAGE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::SYSTEM_RESOURCE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::COMMUNICATIONS);
    tester->enable_diagnostics(diagnostic_types);
    EXPECT_TRUE(tester->get_logger()->log_warn("A Log to Write") ==
                eros::Logger::LoggerStatus::LOG_WRITTEN);

    eros::eros_diagnostic::Diagnostic diag = tester->finish_initialization();
    logger->log_diagnostic(diag);
    EXPECT_TRUE(diag.level <= eros::Level::Type::NOTICE);

    tester->reset();

    double timeToRun = 10.0;
    double dt = 0.1;
    double timer = 0.0;
    while (timer <= timeToRun) {
        diag = tester->update(dt, timer);
        EXPECT_TRUE(diag.level <= eros::Level::Type::NOTICE);
        timer += dt;
    }

    logger->log_warn("Testing Unsupported Program Variables Check");
    {
        std::vector<eros::eros_diagnostic::Diagnostic> diag_list = tester->check_programvariables();
        EXPECT_EQ(diag_list.size(), 0);
    }
    tester->cleanup();

    delete logger;
    delete tester;
}
TEST(BasicTest, TestNotArmed) {
    eros::Logger* logger = new eros::Logger("DEBUG", "UnitTestTankDriveNodeProcess");
    TankDriveNodeProcessTester* tester = new TankDriveNodeProcessTester;
    tester->initialize("UnitTestTankDriveNodeProcess",
                       "UnitTestTankDriveNodeProcess",
                       "MyHost",
                       eros::System::MainSystem::SIMROVER,
                       eros::System::SubSystem::ENTIRE_SYSTEM,
                       eros::System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<eros::eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::SOFTWARE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::DATA_STORAGE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::SYSTEM_RESOURCE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::COMMUNICATIONS);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::REMOTE_CONTROL);
    double current_time = 0.0;
    tester->enable_diagnostics(diagnostic_types);
    EXPECT_TRUE(tester->get_logger()->log_warn("A Log to Write") ==
                eros::Logger::LoggerStatus::LOG_WRITTEN);

    eros::eros_diagnostic::Diagnostic diag = tester->finish_initialization();
    logger->log_diagnostic(diag);
    EXPECT_TRUE(diag.level <= eros::Level::Type::NOTICE);
    EXPECT_EQ(tester->get_mode(), TankDriveNodeProcess::Mode::SIMPLE_ARCADE);
    eros::ArmDisarm::State armed_state;
    armed_state.state = eros::ArmDisarm::Type::DISARMED;
    tester->update_armedstate(armed_state);
    auto left_drive_config = tester->get_left_drive_config();
    auto right_drive_config = tester->get_right_drive_config();
    {  // Max Forward--> No Output (DISARMED)
        geometry_msgs::Twist cmd_vel_perc;
        cmd_vel_perc.linear.x = 100.0;
        cmd_vel_perc.angular.z = 0.0;
        auto output = tester->new_cmd_vel(cmd_vel_perc);
        EXPECT_EQ(output.left_drive.data, left_drive_config.neutral_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.neutral_value);
        tester->update(0.1, current_time + 0.1);
        output = tester->get_drive_command();
        EXPECT_EQ(output.left_drive.data, left_drive_config.neutral_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.neutral_value);
    }
    armed_state.state = eros::ArmDisarm::Type::ARMED;
    tester->update_armedstate(armed_state);
    tester->update(0.1, current_time + 0.1);
    {
        auto output = tester->get_drive_command();
        EXPECT_EQ(output.left_drive.data, left_drive_config.neutral_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.neutral_value);
    }
    {  // Max Forward
        geometry_msgs::Twist cmd_vel_perc;
        cmd_vel_perc.linear.x = 100.0;
        cmd_vel_perc.angular.z = 0.0;
        auto output = tester->new_cmd_vel(cmd_vel_perc);
        EXPECT_EQ(output.left_drive.data, left_drive_config.max_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.min_value);
    }
    armed_state.state = eros::ArmDisarm::Type::DISARMED;
    tester->update_armedstate(armed_state);
    tester->update(0.1, current_time + 0.1);
    {
        auto output = tester->get_drive_command();
        EXPECT_EQ(output.left_drive.data, left_drive_config.neutral_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.neutral_value);
    }
    armed_state.state = eros::ArmDisarm::Type::ARMED;
    tester->update_armedstate(armed_state);
    tester->update(0.1, current_time + 0.1);
    {
        auto output = tester->get_drive_command();
        EXPECT_EQ(output.left_drive.data, left_drive_config.neutral_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.neutral_value);
    }
}
TEST(BasicTest, TestArcadeDriveComputation) {
    eros::Logger* logger = new eros::Logger("DEBUG", "UnitTestTankDriveNodeProcess");
    TankDriveNodeProcessTester* tester = new TankDriveNodeProcessTester;
    tester->initialize("UnitTestTankDriveNodeProcess",
                       "UnitTestTankDriveNodeProcess",
                       "MyHost",
                       eros::System::MainSystem::SIMROVER,
                       eros::System::SubSystem::ENTIRE_SYSTEM,
                       eros::System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<eros::eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::SOFTWARE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::DATA_STORAGE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::SYSTEM_RESOURCE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::COMMUNICATIONS);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::REMOTE_CONTROL);
    tester->enable_diagnostics(diagnostic_types);
    EXPECT_TRUE(tester->get_logger()->log_warn("A Log to Write") ==
                eros::Logger::LoggerStatus::LOG_WRITTEN);

    eros::eros_diagnostic::Diagnostic diag = tester->finish_initialization();
    auto left_drive_config = tester->get_left_drive_config();
    auto right_drive_config = tester->get_right_drive_config();
    logger->log_diagnostic(diag);
    EXPECT_TRUE(diag.level <= eros::Level::Type::NOTICE);
    EXPECT_EQ(tester->get_mode(), TankDriveNodeProcess::Mode::SIMPLE_ARCADE);
    eros::ArmDisarm::State armed_state;
    armed_state.state = eros::ArmDisarm::Type::ARMED;
    tester->update_armedstate(armed_state);
    {  // Stop
        geometry_msgs::Twist cmd_vel_perc;
        cmd_vel_perc.linear.x = 0.0;
        cmd_vel_perc.angular.z = 0.0;
        auto output = tester->new_cmd_vel(cmd_vel_perc);
        EXPECT_EQ(output.left_drive.data, left_drive_config.neutral_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.neutral_value);
    }
    {  // Max Forward
        geometry_msgs::Twist cmd_vel_perc;
        cmd_vel_perc.linear.x = 100.0;
        cmd_vel_perc.angular.z = 0.0;
        auto output = tester->new_cmd_vel(cmd_vel_perc);
        EXPECT_EQ(output.left_drive.data, left_drive_config.max_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.min_value);
    }
    {  // Max Reverse
        geometry_msgs::Twist cmd_vel_perc;
        cmd_vel_perc.linear.x = -100.0;
        cmd_vel_perc.angular.z = 0.0;
        auto output = tester->new_cmd_vel(cmd_vel_perc);
        EXPECT_EQ(output.left_drive.data, left_drive_config.min_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.max_value);
    }
    {  // Max Left
        geometry_msgs::Twist cmd_vel_perc;
        cmd_vel_perc.linear.x = 0;
        cmd_vel_perc.angular.z = 100.0;
        auto output = tester->new_cmd_vel(cmd_vel_perc);
        EXPECT_EQ(output.left_drive.data, left_drive_config.min_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.min_value);
    }
    {  // Max Right
        geometry_msgs::Twist cmd_vel_perc;
        cmd_vel_perc.linear.x = 0;
        cmd_vel_perc.angular.z = -100.0;
        auto output = tester->new_cmd_vel(cmd_vel_perc);
        EXPECT_EQ(output.left_drive.data, left_drive_config.max_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.max_value);
    }

    {  // Max Forward Left
        geometry_msgs::Twist cmd_vel_perc;
        cmd_vel_perc.linear.x = 100.0;
        cmd_vel_perc.angular.z = 100.0;
        auto output = tester->new_cmd_vel(cmd_vel_perc);
        EXPECT_EQ(output.left_drive.data, left_drive_config.neutral_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.min_value);
    }
    {  // Max Forward Right
        geometry_msgs::Twist cmd_vel_perc;
        cmd_vel_perc.linear.x = 100.0;
        cmd_vel_perc.angular.z = -100.0;
        auto output = tester->new_cmd_vel(cmd_vel_perc);
        EXPECT_EQ(output.left_drive.data, left_drive_config.max_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.neutral_value);
    }
    {  // Max Reverse Left
        geometry_msgs::Twist cmd_vel_perc;
        cmd_vel_perc.linear.x = -100.0;
        cmd_vel_perc.angular.z = 100.0;
        auto output = tester->new_cmd_vel(cmd_vel_perc);
        EXPECT_EQ(output.left_drive.data, left_drive_config.min_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.neutral_value);
    }
    {  // Max Reverse Right
        geometry_msgs::Twist cmd_vel_perc;
        cmd_vel_perc.linear.x = -100.0;
        cmd_vel_perc.angular.z = -100.0;
        auto output = tester->new_cmd_vel(cmd_vel_perc);
        EXPECT_EQ(output.left_drive.data, left_drive_config.neutral_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.max_value);
    }
    EXPECT_TRUE(
        tester->get_diagnostic(eros::eros_diagnostic::DiagnosticType::REMOTE_CONTROL).level <
        eros::Level::Type::WARN);
}
TEST(BasicTest, TestArcadeDriveComputation_Invert) {
    eros::Logger* logger = new eros::Logger("DEBUG", "UnitTestTankDriveNodeProcess");
    TankDriveNodeProcessTester* tester = new TankDriveNodeProcessTester;
    tester->initialize("UnitTestTankDriveNodeProcess",
                       "UnitTestTankDriveNodeProcess",
                       "MyHost",
                       eros::System::MainSystem::SIMROVER,
                       eros::System::SubSystem::ENTIRE_SYSTEM,
                       eros::System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<eros::eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::SOFTWARE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::DATA_STORAGE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::SYSTEM_RESOURCE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::COMMUNICATIONS);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::REMOTE_CONTROL);
    tester->enable_diagnostics(diagnostic_types);
    EXPECT_TRUE(tester->get_logger()->log_warn("A Log to Write") ==
                eros::Logger::LoggerStatus::LOG_WRITTEN);

    eros::eros_diagnostic::Diagnostic diag = tester->finish_initialization();
    TankDriveNodeProcess::DriveChannelConfig left_drive_config;
    TankDriveNodeProcess::DriveChannelConfig right_drive_config;
    left_drive_config.min_value = 2000;
    left_drive_config.neutral_value = 1500;
    left_drive_config.max_value = 1000;
    right_drive_config.min_value = 2000;
    right_drive_config.neutral_value = 1500;
    right_drive_config.max_value = 1000;
    tester->set_left_drive_config(left_drive_config);
    tester->set_right_drive_config(right_drive_config);
    logger->log_diagnostic(diag);
    EXPECT_TRUE(diag.level <= eros::Level::Type::NOTICE);
    EXPECT_EQ(tester->get_mode(), TankDriveNodeProcess::Mode::SIMPLE_ARCADE);
    eros::ArmDisarm::State armed_state;
    armed_state.state = eros::ArmDisarm::Type::ARMED;
    tester->update_armedstate(armed_state);
    {  // Stop
        geometry_msgs::Twist cmd_vel_perc;
        cmd_vel_perc.linear.x = 0.0;
        cmd_vel_perc.angular.z = 0.0;
        auto output = tester->new_cmd_vel(cmd_vel_perc);
        EXPECT_EQ(output.left_drive.data, left_drive_config.neutral_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.neutral_value);
    }
    {  // Max Forward
        geometry_msgs::Twist cmd_vel_perc;
        cmd_vel_perc.linear.x = 100.0;
        cmd_vel_perc.angular.z = 0.0;
        auto output = tester->new_cmd_vel(cmd_vel_perc);
        EXPECT_EQ(output.left_drive.data, left_drive_config.max_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.min_value);
    }

    {  // Max Reverse
        geometry_msgs::Twist cmd_vel_perc;
        cmd_vel_perc.linear.x = -100.0;
        cmd_vel_perc.angular.z = 0.0;
        auto output = tester->new_cmd_vel(cmd_vel_perc);
        EXPECT_EQ(output.left_drive.data, left_drive_config.min_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.max_value);
    }
    {  // Max Left
        geometry_msgs::Twist cmd_vel_perc;
        cmd_vel_perc.linear.x = 0;
        cmd_vel_perc.angular.z = 100.0;
        auto output = tester->new_cmd_vel(cmd_vel_perc);
        EXPECT_EQ(output.left_drive.data, left_drive_config.min_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.min_value);
    }
    {  // Max Right
        geometry_msgs::Twist cmd_vel_perc;
        cmd_vel_perc.linear.x = 0;
        cmd_vel_perc.angular.z = -100.0;
        auto output = tester->new_cmd_vel(cmd_vel_perc);
        EXPECT_EQ(output.left_drive.data, left_drive_config.max_value);
        EXPECT_EQ(output.right_drive.data, right_drive_config.max_value);
    }

    EXPECT_TRUE(
        tester->get_diagnostic(eros::eros_diagnostic::DiagnosticType::REMOTE_CONTROL).level <
        eros::Level::Type::WARN);
}
TEST(TestCommands, TestAllCommands) {
    eros::Logger* logger = new eros::Logger("DEBUG", "UnitTestTankDriveNodeProcess");
    TankDriveNodeProcessTester* tester = new TankDriveNodeProcessTester;
    tester->initialize("UnitTestTankDriveNodeProcess",
                       "UnitTestTankDriveNodeProcess",
                       "MyHost",
                       eros::System::MainSystem::SIMROVER,
                       eros::System::SubSystem::ENTIRE_SYSTEM,
                       eros::System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<eros::eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::SOFTWARE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::DATA_STORAGE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::SYSTEM_RESOURCE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::COMMUNICATIONS);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::REMOTE_CONTROL);
    tester->enable_diagnostics(diagnostic_types);
    EXPECT_TRUE(tester->get_logger()->log_warn("A Log to Write") ==
                eros::Logger::LoggerStatus::LOG_WRITTEN);

    eros::eros_diagnostic::Diagnostic diag = tester->finish_initialization();
    EXPECT_TRUE(diag.level <= eros::Level::Type::NOTICE);
    for (uint8_t i = (uint16_t)eros::Command::Type::UNKNOWN;
         i < (uint16_t)eros::Command::Type::END_OF_LIST;
         ++i) {
        eros::command new_cmd;
        new_cmd.Command = i;
        std::vector<eros::eros_diagnostic::Diagnostic> diag_list = tester->new_commandmsg(new_cmd);
        EXPECT_GT(diag_list.size(), 0);
        for (auto diag : diag_list) { EXPECT_TRUE(diag.level < eros::Level::Type::WARN); }
    }

    delete tester;
    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
