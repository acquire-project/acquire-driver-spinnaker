# Spinnaker SDK library

find_path(spinnaker_include_dir "Spinnaker.h"
    PATH_SUFFIXES
        "include/spinnaker"  # osx
        "FLIR Systems/Spinnaker/include"  # windows
    DOC "Directory that contains Spinnaker.h"
    NO_CACHE)

if(spinnaker_include_dir)
    message(STATUS "Spinnaker ${spinnaker_include_dir}")

    set(tgt spinnaker)
    add_library(${tgt} SHARED IMPORTED)
    target_include_directories(${tgt} INTERFACE ${spinnaker_include_dir})

    if(WIN32)
        # TODO: set imported_implib for windows or link to dll.
        # https://cmake.org/cmake/help/latest/prop_tgt/IMPORTED_IMPLIB.html
        set_target_properties(${tgt} PROPERTIES
            IMPORTED_LOCATION "${spinnaker_include_dir}../../lib/spinnaker.lib"
        )
    elseif(APPLE)
        set_target_properties(${tgt} PROPERTIES
            IMPORTED_LOCATION "${spinnaker_include_dir}../../lib/libSpinnaker.dylib"
        )
    endif()
else()
    message(STATUS "Could not find Spinnaker.h")
endif()
