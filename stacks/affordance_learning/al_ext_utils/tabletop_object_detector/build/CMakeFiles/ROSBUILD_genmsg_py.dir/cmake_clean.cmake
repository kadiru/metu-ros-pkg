FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/tabletop_object_detector/msg"
  "../src/tabletop_object_detector/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/tabletop_object_detector/msg/__init__.py"
  "../src/tabletop_object_detector/msg/_Table.py"
  "../src/tabletop_object_detector/msg/_TabletopDetectionResult.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
