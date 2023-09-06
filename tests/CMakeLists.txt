if(${NOTEST})
        message(STATUS "Skipping test targets")
else()
        set(NOTEST "TRUE")
        aq_require(acquire-driver-common)
        aq_require(acquire-video-runtime)
        set(NOTEST "FALSE")

        #
        # PARAMETERS
        #
        set(project acquire-driver-spinnaker) # CMAKE_PROJECT_NAME gets overridden if this is a subtree of another project

        #
        # Tests
        #
        set(tests
                list-devices
                one-video-stream
                repeat-start
                repeat-start-no-stop
                configure-properties
                configure-triggering
                abort-while-waiting-for-trigger
        )

        foreach(name ${tests})
                set(tgt "${project}-${name}")
                add_executable(${tgt} ${name}.cpp)
                target_compile_definitions(${tgt} PUBLIC "TEST=\"${tgt}\"")
                set_target_properties(${tgt} PROPERTIES
                        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
                )
                target_include_directories(${tgt} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../")
                target_link_libraries(${tgt}
                        acquire-core-logger
                        acquire-core-platform
                        acquire-video-runtime
                )

                add_test(NAME test-${tgt} COMMAND ${tgt})
                set_tests_properties(test-${tgt} PROPERTIES LABELS acquire-driver-spinnaker)
        endforeach()

        #
        # Copy driver to tests
        #
        list(POP_FRONT tests onename)

        foreach(driver
                acquire-driver-common
                acquire-driver-spinnaker
        )
                add_custom_target(${project}-copy-${driver}-for-tests
                        COMMAND ${CMAKE_COMMAND} -E copy
                        $<TARGET_FILE:${driver}>
                        $<TARGET_FILE_DIR:${project}-${onename}>
                        DEPENDS ${driver}
                        COMMENT "Copying ${driver} to $<TARGET_FILE_DIR:${project}-${onename}>"
                )

                foreach(name ${tests})
                        add_dependencies(${tgt} ${project}-copy-${driver}-for-tests)
                endforeach()
        endforeach()
endif()
