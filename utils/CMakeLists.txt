set(project acquire-driver-spinnaker)
set(tgt set-packet-size)
add_executable(${tgt} set-packet-size.cpp) 
target_link_libraries(${tgt} spinnaker)