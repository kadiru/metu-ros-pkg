FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/tabletop_object_detector/msg"
  "../src/tabletop_object_detector/srv"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/tabletop_object_detector/Table.h"
  "../msg_gen/cpp/include/tabletop_object_detector/TabletopDetectionResult.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
