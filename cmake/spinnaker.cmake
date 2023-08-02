# Spinnaker SDK library

find_path(spinnaker_include_dir "Spinnaker.h"
    PATH_SUFFIXES "spinnaker"
    DOC "Directory that contains Spinnaker.h"
    NO_CACHE)

if(spinnaker_include_dir)
    message(STATUS "Spinnaker ${spinnaker_include_dir}")

    set(tgt spinnaker)
    add_library(${tgt} IMPORTED INTERFACE)
    target_include_directories(${tgt} INTERFACE ${spinnaker_include_dir})
    target_link_directories(${tgt} INTERFACE ${spinnaker_include_dir}/../../lib)
    target_link_libraries(${tgt} INTERFACE libSpinnaker.dylib)
endif()
