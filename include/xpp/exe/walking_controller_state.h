/**
 @file    walking_controller_state.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 30, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_EXE_WALKING_CONTROLLER_STATE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_EXE_WALKING_CONTROLLER_STATE_H_

#include <map>

namespace xpp {
namespace exe {

class WalkingController;

class WalkingControllerState {
public:
  enum TaskState{PLANNING, EXECUTING, WAITING};
  typedef std::map<TaskState, WalkingControllerState*> StatesMap;

  WalkingControllerState ();
  virtual ~WalkingControllerState ();

  virtual void Run(WalkingController*) const = 0;
  static StatesMap BuildStates();
};

class Planning : public WalkingControllerState {
public:
  void Run(WalkingController* context) const;
};

class Executing : public WalkingControllerState {
public:
  void Run(WalkingController* context) const;
};

} /* namespace exe */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_EXE_WALKING_CONTROLLER_STATE_H_ */