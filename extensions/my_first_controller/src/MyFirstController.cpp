#include "MyFirstController.h"
#include <mc_rbdyn/RobotLoader.h>

MyFirstController::MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{

  config_.load(config);
  solver().addConstraintSet(contactConstraint);

  // Set left foot and right foot contact to the surface
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});

  /* Self Collision Avoidance*/
  solver().addConstraintSet(selfCollisionConstraint);
  /* Joint and Velocity limits*/
  solver().addConstraintSet(kinematicsConstraint);

  solver().addTask(postureTask);

  comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0);
  solver().addTask(comTask);

  lTurn = std::make_shared<mc_tasks::LookAtFrameTask>(robots(), 
  robot().robotIndex(), headCenter, Eigen::Vector3d{0, 0, 1.0}, robot().robotIndex(), lHand, 10);
  // solver().addTask(lTurn);

  rTurn = std::make_shared<mc_tasks::LookAtFrameTask>(robots(), 
  robot().robotIndex(), headCenter, Eigen::Vector3d{0, 0, 1.0}, robot().robotIndex(), rHand, 10);
  // solver().addTask(rTurn);

  std::vector<std::string> rightJoints = {"R_SHOULDER_P", "R_SHOULDER_Y", "R_SHOULDER_R",
    "R_ELBOW_P", "R_ELBOW_Y", "R_WRIST_R", "R_WRIST_Y", "R_UTHUMB" };
  
  std::vector<std::string> leftJoints = {"L_SHOULDER_P", "L_SHOULDER_Y", "L_SHOULDER_R",
      "L_ELBOW_P", "L_ELBOW_Y", "L_WRIST_R", "L_WRIST_Y", "L_UTHUMB"};
  
  /* Left hand movement task*/
  lfTask = std::make_shared<mc_tasks::EndEffectorTask>("l_wrist", robots(), 0, 1.0, 500.0);
  lfTask->selectActiveJoints(solver(), leftJoints);
  // lfTask->selectUnactiveJoints(solver(), rightJoints);
  // solver().addTask(lfTask);
  
  // /* Right hand movement task*/
  rhTask = std::make_shared<mc_tasks::EndEffectorTask>("r_wrist", robots(), 0, 1.0, 500.0);
  rhTask->selectActiveJoints(solver(), rightJoints);
  // rhTask->selectUnactiveJoints(solver(), leftJoints);
  // solver().addTask(rhTask);

  mc_rtc::log::success("MyFirstController init done ");
}

bool MyFirstController::run()
{
  // bool ret = mc_control::MCController::run();
  auto lpt = lfTask->get_ef_pose();
  auto rpt = rhTask->get_ef_pose();
  // auto p = postureTask->posture();

  sva::PTransformd leftPose{
    Eigen::Quaterniond{0, 0.7, 0, 0.7}, Eigen::Vector3d{0.5, 0.25, 1.1}};

  sva::PTransformd rightPose{
      Eigen::Quaterniond{0, 0.7, 0, 0.7}, Eigen::Vector3d{0.5, -0.25, 1.1}};

    if(!move_Left){

      if(postureTask->eval().norm() < 5e-2 && comTask->eval().norm() < 0.05)
      {
        mc_rtc::log::success("Moved left hand");
        move_Left = true;
        auto loffset = lpt.inv() * leftPose;
        lfTask->set_ef_pose(loffset);
        solver().addTask(lfTask);
        solver().addTask(lTurn);
      }
      // return ret;
    }

    if(move_Left && !move_Right)
    {
      if (lfTask->eval().norm() < 5e-2 && lfTask->speed().norm() < 1e-4 && comTask->eval().norm() < 0.05)
      {
        mc_rtc::log::success("Moved right hand");
        move_Right = true;
        solver().removeTask(lTurn);
        lfTask->set_ef_pose(lfTask->get_ef_pose().inv() * lpt);
        solver().removeTask(lfTask);

        auto roffset = rpt.inv() * rightPose;
        rhTask->set_ef_pose(roffset); 
        solver().addTask(rhTask);
        solver().addTask(rTurn);     
      }
      // return ret;
    }

    if (move_Left && move_Right && !move_both)
    {
      if (rhTask->eval().norm() < 5e-2 && rhTask->speed().norm() < 1e-4)
      {
        mc_rtc::log::success("Moved both hands");
        move_both = true;
        // postureTask->posture(p);
        solver().removeTask(rTurn);
        rhTask->set_ef_pose(rhTask->get_ef_pose().inv() * rpt);
        solver().removeTask(rhTask);

        solver().addTask(lfTask);
        lfTask->set_ef_pose(lpt.inv() * leftPose);
        
        solver().addTask(rhTask);
        rhTask->set_ef_pose(rpt.inv() * rightPose);
      }
      // return ret;
    }

  
  // if(std::abs(postureTask->posture()[jointIndex][0] - robot().mbc().q[jointIndex][0]) < 0.05) { switch_target(); }
  // auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  // for(const auto & j : rm->mb.joints())
  // {
  //   if(j.dof() == 1 && !j.isMimic()) 
  //   { std::cout << "- " << j.name() << "\n"; 
  //     std::cout<<std::endl;
  //   }
  // }
  
  return mc_control::MCController::run();
}

void MyFirstController::reset(const mc_control::ControllerResetData & reset_data)
{
  // comZero = comTask->com();
  comTask->reset();

  lTurn->reset();

  lfTask->reset();
  lfTask->resetJointsSelector(solver());

  rTurn->reset();

  rhTask->reset();
  rhTask->resetJointsSelector(solver());
  postureTask->reset();

  // move_Left = !move_Left;
  // move_Right = !move_Right;
  // move_both = !move_both;

  mc_control::MCController::reset(reset_data);
}

void MyFirstController::switch_target()
{
  // auto lpt = lfTask->get_ef_pose();
  // auto rpt = rhTask->get_ef_pose();
  // auto p = postureTask->posture();

  // if(all_tasks)
  // {

  //   if(!move_Left){

  //     if(comTask->eval().norm() < 5e-2 && comTask->speed().norm() < 1e-4)
  //     {
  //       mc_rtc::log::success("Moved left hand");
  //       move_Left = true;
  //       // solver().addTask(lfTask); 
  //       postureTask->target({{jointName, robot().qu()[jointIndex]}});
  //       lfTask->set_ef_pose(sva::PTransformd{
  //         Eigen::Quaterniond{0, 0.7, 0, 0.7}, Eigen::Vector3d{0.5, 0.25, 1.1}});
  //     }
  //     // return ret;
  //   }

  //   if(move_Left && !move_Right)
  //   {
  //     if (lfTask->eval().norm() < 5e-2 && lfTask->speed().norm() < 1e-4)
  //     {
  //       mc_rtc::log::success("Moved right hand");
  //       move_Right = true;
  //       lfTask->set_ef_pose(lpt);
  //       // solver().removeTask(lfTask);

  //       // solver().addTask(rhTask);
  //       postureTask->target({{jointName, robot().ql()[jointIndex]}}); 
  //       rhTask->set_ef_pose(sva::PTransformd{
  //         Eigen::Quaterniond{0, 0.7, 0, 0.7}, Eigen::Vector3d{0.5, -0.25, 1.1}});      
  //     }
  //     // return ret;
  //   }

  //   if (move_Left && move_Right && !move_both)
  //   {
  //     if (rhTask->eval().norm() < 5e-2 && rhTask->speed().norm() < 1e-4)
  //     {
  //       mc_rtc::log::success("Moved both hands");
  //       move_both = true;
  //       postureTask->posture(p);
  //       rhTask->set_ef_pose(rpt);

  //       // solver().addTask(lfTask);
  //       lfTask->set_ef_pose(sva::PTransformd{
  //         Eigen::Quaterniond{0, 0.7, 0, 0.7}, Eigen::Vector3d{0.5, 0.25, 1.1}});
        
  //       rhTask->set_ef_pose(sva::PTransformd{
  //         Eigen::Quaterniond{0, 0.7, 0, 0.7}, Eigen::Vector3d{0.5, -0.25, 1.1}});
  //     }
  //     // return ret;
  //   }
  // }

  // all_tasks = !all_tasks;
    
}

CONTROLLER_CONSTRUCTOR("MyFirstController", MyFirstController)
