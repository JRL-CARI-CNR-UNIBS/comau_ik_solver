#include <comau_ik_solver/comau_ik_solver.h>
#include <comau_ik_solver_ros/comau_ik_solver_ros.h>


PLUGINLIB_EXPORT_CLASS(ik_solver::NJ220_27_IkSolver, ik_solver::IkSolver);
PLUGINLIB_EXPORT_CLASS(ik_solver::NJ370_27_IkSolver, ik_solver::IkSolver);
PLUGINLIB_EXPORT_CLASS(ik_solver::NJ_IkSolver      , ik_solver::IkSolver);

PLUGINLIB_EXPORT_CLASS(ik_solver::NJ220_27_IkSolverOptions, ik_solver::IkSolverOptions);
PLUGINLIB_EXPORT_CLASS(ik_solver::NJ370_27_IkSolverOptions, ik_solver::IkSolverOptions);
PLUGINLIB_EXPORT_CLASS(ik_solver::NJ_IkSolverOptions      , ik_solver::IkSolverOptions);
