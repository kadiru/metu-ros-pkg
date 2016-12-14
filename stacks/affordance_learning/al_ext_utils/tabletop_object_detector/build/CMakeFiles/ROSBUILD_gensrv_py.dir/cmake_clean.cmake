FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/tabletop_object_detector/msg"
  "../src/tabletop_object_detector/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/tabletop_object_detector/srv/__init__.py"
  "../src/tabletop_object_detector/srv/_TabletopSegmentation.py"
  "../src/tabletop_object_detector/srv/_TabletopObjectRecognition.py"
  "../src/tabletop_object_detector/srv/_NegateExclusions.py"
  "../src/tabletop_object_detector/srv/_SegmentObjectInHand.py"
  "../src/tabletop_object_detector/srv/_AddModelExclusion.py"
  "../src/tabletop_object_detector/srv/_ClearExclusionsList.py"
  "../src/tabletop_object_detector/srv/_TabletopDetection.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
