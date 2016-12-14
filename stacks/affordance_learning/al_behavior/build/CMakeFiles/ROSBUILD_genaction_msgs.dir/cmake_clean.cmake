FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/al_behavior/msg"
  "CMakeFiles/ROSBUILD_genaction_msgs"
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
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
