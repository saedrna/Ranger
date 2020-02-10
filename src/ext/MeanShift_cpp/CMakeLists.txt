file(GLOB MEANSHIFT_CPP_SRC "*.h" "*.cpp")

add_library(MeanShift_cpp STATIC ${MEANSHIFT_CPP_SRC})
target_include_directories(MeanShift_cpp PUBLIC ${CMAKE_CURRENT_LIST_DIR})
