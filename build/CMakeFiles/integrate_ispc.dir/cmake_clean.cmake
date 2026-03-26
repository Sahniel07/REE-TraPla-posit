file(REMOVE_RECURSE
  "integrate_ispc.h"
  "integrate_ispc.o"
  "libintegrate_ispc.a"
  "libintegrate_ispc.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/integrate_ispc.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
