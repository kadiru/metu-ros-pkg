FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/tabletop_collision_map_processing/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/tabletop_collision_map_processing/srv/__init__.py"
  "../src/tabletop_collision_map_processing/srv/_TabletopCollisionMapProcessing.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
