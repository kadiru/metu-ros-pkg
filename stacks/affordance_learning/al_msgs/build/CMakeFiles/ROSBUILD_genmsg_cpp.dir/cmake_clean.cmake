FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/al_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/al_msgs/Feature.h"
  "../msg_gen/cpp/include/al_msgs/JointCmd.h"
  "../msg_gen/cpp/include/al_msgs/Entity.h"
  "../msg_gen/cpp/include/al_msgs/CloudObjects.h"
  "../msg_gen/cpp/include/al_msgs/Affordances.h"
  "../msg_gen/cpp/include/al_msgs/Object.h"
  "../msg_gen/cpp/include/al_msgs/Speech.h"
  "../msg_gen/cpp/include/al_msgs/Entities.h"
  "../msg_gen/cpp/include/al_msgs/SceneObjects.h"
  "../msg_gen/cpp/include/al_msgs/Scene.h"
  "../msg_gen/cpp/include/al_msgs/Table.h"
  "../msg_gen/cpp/include/al_msgs/Spec.h"
  "../msg_gen/cpp/include/al_msgs/Features.h"
  "../msg_gen/cpp/include/al_msgs/Shape.h"
  "../msg_gen/cpp/include/al_msgs/Effect.h"
  "../msg_gen/cpp/include/al_msgs/Shapes.h"
  "../msg_gen/cpp/include/al_msgs/FeatureVectorVector.h"
  "../msg_gen/cpp/include/al_msgs/Color.h"
  "../msg_gen/cpp/include/al_msgs/Behavior.h"
  "../msg_gen/cpp/include/al_msgs/PointIndices.h"
  "../msg_gen/cpp/include/al_msgs/AffordancesComp.h"
  "../msg_gen/cpp/include/al_msgs/SceneAffordances.h"
  "../msg_gen/cpp/include/al_msgs/CollisionObjects.h"
  "../msg_gen/cpp/include/al_msgs/FeatureVector.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
