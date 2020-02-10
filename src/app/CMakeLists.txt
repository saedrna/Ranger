file(GLOB SRC "*.cpp")

add_executable(ranger ${SRC})

target_link_libraries(ranger Mosek::Mosek fusion_cxx MeanShift_cpp)