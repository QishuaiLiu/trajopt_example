include_directories(include)
add_executable(trajopt_example src/trajopt_example.cpp)
target_link_libraries(trajopt_example  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(fetch_trajopt_example 
    src/fetch_trajopt_example.cpp 
    src/utils.cpp
    src/geometry.cpp
    src/yaml.cpp)

target_link_libraries(fetch_trajopt_example  ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

install(TARGETS trajopt_example DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
