#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include<mc_tasks/SurfaceTransformTask.h>
#include <mc_tasks/LookAtFrameTask.h>
#include <mc_tasks/CoMTask.h>

#include "api.h"

struct MyFirstController_DLLAPI MyFirstController : public mc_control::MCController
{

  MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  void switch_target();

private:
  mc_rtc::Configuration config_;
  // std::string jointName = "NECK_Y";
  std::string headCenter = "dcamera", lHand = "l_wrist", rHand = "r_wrist";
  // lHand = "LeftGripper", rHand = "RightGripper";

  bool move_Left = false, move_Right = false, move_both = false;
  bool all_tasks = true;

  // std::shared_ptr<mc_tasks::SurfaceTransformTask> lfTask;
  std::shared_ptr<mc_tasks::EndEffectorTask> lfTask;
  std::shared_ptr<mc_tasks::EndEffectorTask> rhTask;
  std::shared_ptr<mc_tasks::CoMTask> comTask;
  std::shared_ptr<mc_tasks::LookAtFrameTask> lTurn;
  std::shared_ptr<mc_tasks::LookAtFrameTask> rTurn;
};
