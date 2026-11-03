cmake_minimum_required(VERSION 3.16)
project(micromouse_algo CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(mm_core
  src/lib/maze_io.cpp
)
target_include_directories(mm_core PUBLIC include)
target_compile_options(mm_core PRIVATE -Wall -Wextra)

add_executable(run src/apps/run.cpp)
target_link_libraries(run PRIVATE mm_core)
target_compile_options(run PRIVATE -Wall -Wextra)

# binary to point the mms simulator GUI at
add_executable(mms src/apps/mms_main.cpp)
target_link_libraries(mms PRIVATE mm_core)
target_compile_options(mms PRIVATE -Wall -Wextra)

# quick unit smoke test
add_executable(test_core tests/test_core.cpp)
target_link_libraries(test_core PRIVATE mm_core)
target_compile_options(test_core PRIVATE -Wall -Wextra)
