cmake_minimum_required(VERSION 3.13)
project(acquire-driver-spinnaker)
cmake_policy(SET CMP0079 NEW) # allows use with targets in other directories
enable_testing()

include(cmake/aq_require.cmake)
include(cmake/git-versioning.cmake)
include(cmake/spinnaker.cmake)
include(cmake/ide.cmake)
include(cmake/install-prefix.cmake)
include(cmake/simd.cmake)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

add_subdirectory(src)
add_subdirectory(utils)
add_subdirectory(tests)

include(CPack)
