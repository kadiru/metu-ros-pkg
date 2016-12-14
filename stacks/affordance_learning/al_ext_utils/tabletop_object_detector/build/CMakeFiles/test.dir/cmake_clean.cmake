FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/tabletop_object_detector/msg"
  "../src/tabletop_object_detector/srv"
  "CMakeFiles/test"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/test.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
