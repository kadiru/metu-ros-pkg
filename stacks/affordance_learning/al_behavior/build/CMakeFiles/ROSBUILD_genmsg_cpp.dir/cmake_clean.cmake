FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/al_behavior/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/al_behavior/ArmAction.h"
  "../msg_gen/cpp/include/al_behavior/ArmGoal.h"
  "../msg_gen/cpp/include/al_behavior/ArmActionGoal.h"
  "../msg_gen/cpp/include/al_behavior/ArmResult.h"
  "../msg_gen/cpp/include/al_behavior/ArmActionResult.h"
  "../msg_gen/cpp/include/al_behavior/ArmFeedback.h"
  "../msg_gen/cpp/include/al_behavior/ArmActionFeedback.h"
  "../msg_gen/cpp/include/al_behavior/HeadAction.h"
  "../msg_gen/cpp/include/al_behavior/HeadGoal.h"
  "../msg_gen/cpp/include/al_behavior/HeadActionGoal.h"
  "../msg_gen/cpp/include/al_behavior/HeadResult.h"
  "../msg_gen/cpp/include/al_behavior/HeadActionResult.h"
  "../msg_gen/cpp/include/al_behavior/HeadFeedback.h"
  "../msg_gen/cpp/include/al_behavior/HeadActionFeedback.h"
  "../msg/ArmAction.msg"
  "../msg/ArmGoal.msg"
  "../msg/ArmActionGoal.msg"
  "../msg/ArmResult.msg"
  "../msg/ArmActionResult.msg"
  "../msg/ArmFeedback.msg"
  "../msg/ArmActionFeedback.msg"
  "../msg/HeadAction.msg"
  "../msg/HeadGoal.msg"
  "../msg/HeadActionGoal.msg"
  "../msg/HeadResult.msg"
  "../msg/HeadActionResult.msg"
  "../msg/HeadFeedback.msg"
  "../msg/HeadActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
