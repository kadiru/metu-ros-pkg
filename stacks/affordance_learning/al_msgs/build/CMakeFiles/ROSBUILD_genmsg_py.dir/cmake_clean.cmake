FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/al_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/al_msgs/msg/__init__.py"
  "../src/al_msgs/msg/_Feature.py"
  "../src/al_msgs/msg/_JointCmd.py"
  "../src/al_msgs/msg/_Entity.py"
  "../src/al_msgs/msg/_CloudObjects.py"
  "../src/al_msgs/msg/_Affordances.py"
  "../src/al_msgs/msg/_Object.py"
  "../src/al_msgs/msg/_Speech.py"
  "../src/al_msgs/msg/_Entities.py"
  "../src/al_msgs/msg/_SceneObjects.py"
  "../src/al_msgs/msg/_Scene.py"
  "../src/al_msgs/msg/_Table.py"
  "../src/al_msgs/msg/_Spec.py"
  "../src/al_msgs/msg/_Features.py"
  "../src/al_msgs/msg/_Shape.py"
  "../src/al_msgs/msg/_Effect.py"
  "../src/al_msgs/msg/_Shapes.py"
  "../src/al_msgs/msg/_FeatureVectorVector.py"
  "../src/al_msgs/msg/_Color.py"
  "../src/al_msgs/msg/_Behavior.py"
  "../src/al_msgs/msg/_PointIndices.py"
  "../src/al_msgs/msg/_AffordancesComp.py"
  "../src/al_msgs/msg/_SceneAffordances.py"
  "../src/al_msgs/msg/_CollisionObjects.py"
  "../src/al_msgs/msg/_FeatureVector.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
