FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/al_srvs/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/al_srvs/srv/__init__.py"
  "../src/al_srvs/srv/_GetLearnedBehavior.py"
  "../src/al_srvs/srv/_GetLearnedObjectShapes.py"
  "../src/al_srvs/srv/_Perception.py"
  "../src/al_srvs/srv/_GetScene.py"
  "../src/al_srvs/srv/_GetBehaviorsByEffect.py"
  "../src/al_srvs/srv/_GetLearnedEffect.py"
  "../src/al_srvs/srv/_GetEntityVisualFeatures.py"
  "../src/al_srvs/srv/_GetBehavior.py"
  "../src/al_srvs/srv/_GetEntitiesVisualFeatures.py"
  "../src/al_srvs/srv/_GetEntity.py"
  "../src/al_srvs/srv/_GetCollisionObject.py"
  "../src/al_srvs/srv/_GetLearnedAdjectives.py"
  "../src/al_srvs/srv/_GetLearnedEffects.py"
  "../src/al_srvs/srv/_PerceptionAll.py"
  "../src/al_srvs/srv/_LearnEffect.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
