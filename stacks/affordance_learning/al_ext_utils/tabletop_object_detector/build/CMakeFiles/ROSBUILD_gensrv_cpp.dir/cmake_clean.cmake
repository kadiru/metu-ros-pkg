FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/tabletop_object_detector/msg"
  "../src/tabletop_object_detector/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/tabletop_object_detector/TabletopSegmentation.h"
  "../srv_gen/cpp/include/tabletop_object_detector/TabletopObjectRecognition.h"
  "../srv_gen/cpp/include/tabletop_object_detector/NegateExclusions.h"
  "../srv_gen/cpp/include/tabletop_object_detector/SegmentObjectInHand.h"
  "../srv_gen/cpp/include/tabletop_object_detector/AddModelExclusion.h"
  "../srv_gen/cpp/include/tabletop_object_detector/ClearExclusionsList.h"
  "../srv_gen/cpp/include/tabletop_object_detector/TabletopDetection.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
