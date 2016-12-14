FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/al_srvs/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/al_srvs/GetLearnedBehavior.h"
  "../srv_gen/cpp/include/al_srvs/GetLearnedObjectShapes.h"
  "../srv_gen/cpp/include/al_srvs/Perception.h"
  "../srv_gen/cpp/include/al_srvs/GetScene.h"
  "../srv_gen/cpp/include/al_srvs/GetBehaviorsByEffect.h"
  "../srv_gen/cpp/include/al_srvs/GetLearnedEffect.h"
  "../srv_gen/cpp/include/al_srvs/GetEntityVisualFeatures.h"
  "../srv_gen/cpp/include/al_srvs/GetBehavior.h"
  "../srv_gen/cpp/include/al_srvs/GetEntitiesVisualFeatures.h"
  "../srv_gen/cpp/include/al_srvs/GetEntity.h"
  "../srv_gen/cpp/include/al_srvs/GetCollisionObject.h"
  "../srv_gen/cpp/include/al_srvs/GetLearnedAdjectives.h"
  "../srv_gen/cpp/include/al_srvs/GetLearnedEffects.h"
  "../srv_gen/cpp/include/al_srvs/PerceptionAll.h"
  "../srv_gen/cpp/include/al_srvs/LearnEffect.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
