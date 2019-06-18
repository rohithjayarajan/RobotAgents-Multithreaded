add_executable(roboagents main.cpp robot.cpp world.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
find_package (Threads)
target_link_libraries(roboagents ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
