find_program(GIT git)
set(GIT_TAG "v0")
set(GIT_HASH "")

if(GIT)
    execute_process(COMMAND ${GIT} describe --tags --abbrev=0
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE _GIT_TAG
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    execute_process(COMMAND ${GIT} describe --always
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    string(FIND "${_GIT_TAG}" "fatal" find_result)

    if(${find_result} EQUAL -1)
        set(GIT_TAG "${_GIT_TAG}")
    endif()
endif()

set(CPACK_PACKAGE_VERSION ${GIT_TAG})
message(STATUS "Version ${GIT_TAG} ${GIT_HASH}")
function(target_add_git_versioning tgt)
    target_compile_definitions(${tgt} PRIVATE
        -DGIT_TAG=${GIT_TAG}
        -DGIT_HASH=${GIT_HASH}
    )
endfunction()
