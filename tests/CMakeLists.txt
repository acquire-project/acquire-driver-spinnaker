if(${NOTEST})
        message(STATUS "Skipping test targets")
else()
        set(NOTEST "TRUE")
        add_subdirectory(../acquire-common/acquire-driver-common ${CMAKE_CURRENT_BINARY_DIR}/acquire-driver-common)
        add_subdirectory(../acquire-common/acquire-video-runtime ${CMAKE_CURRENT_BINARY_DIR}/acquire-video-runtime)
        set(NOTEST "FALSE")

        #
        # PARAMETERS
        #
        set(project acquire-driver-spinnaker) # CMAKE_PROJECT_NAME gets overridden if this is a subtree of another project

        #
        # Define setup fixtures
        #
        set(tgt set-packet-size)
        add_executable(${tgt} set-packet-size.cpp)
        target_link_libraries(${tgt} spinnaker)
        # Ensure that the packet size of the Oryx camera is set to 1000 bytes
        # before any of its tests run.
        add_test(NAME test-${tgt} COMMAND ${tgt} ORX-10GS-51S5M 1000)
        set_tests_properties(test-${tgt} PROPERTIES FIXTURES_SETUP setup-oryx)

        #
        # Tests
        #
        set(tests
                list-devices
                blackfly-metadata
                blackfly-one-video-stream
                blackfly-repeat-start
                blackfly-repeat-start-no-stop
                blackfly-configure-properties
                blackfly-configure-triggering
                blackfly-abort-while-waiting-for-trigger
                oryx-metadata
                oryx-one-video-stream
                oryx-repeat-start
                oryx-repeat-start-no-stop
                oryx-configure-properties
                oryx-configure-triggering
                oryx-abort-while-waiting-for-trigger
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
                if(name MATCHES "^oryx")
                        set_tests_properties(test-${tgt} PROPERTIES FIXTURES_REQUIRED setup-oryx)
                        set_tests_properties(test-${tgt} PROPERTIES LABELS "acquire-driver-spinnaker;oryx")
                elseif(name MATCHES "^blackfly")
                        set_tests_properties(test-${tgt} PROPERTIES LABELS "acquire-driver-spinnaker;blackfly")
                else() # list-devices
                        set_tests_properties(test-${tgt} PROPERTIES LABELS "acquire-driver-spinnaker;blackfly;oryx")
                endif()
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
