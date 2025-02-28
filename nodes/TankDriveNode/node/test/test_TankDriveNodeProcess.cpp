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

    logger->log_warn("Testing Unsupported Command Message");
    {
        eros::command cmd;
        std::vector<eros::eros_diagnostic::Diagnostic> diag_list = tester->new_commandmsg(cmd);
        EXPECT_EQ(diag_list.size(), 0);
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
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
