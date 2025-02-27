/*! \file TankDriveNodeProcess.h
 */
#pragma once
#include <eros/BaseNodeProcess.h>
#include <eros_diagnostic/Diagnostic.h>
/*! \class TankDriveNodeProcess TankDriveNodeProcess.h "TankDriveNodeProcess.h"
 *  \brief */
namespace crawler_app {
class TankDriveNodeProcess : public eros::BaseNodeProcess
{
   public:
    TankDriveNodeProcess();
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

   private:
};
}  // namespace crawler_app