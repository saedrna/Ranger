file(GLOB FUSION_CXX_SRC "*.h" "*.cc")

add_library(fusion_cxx STATIC ${FUSION_CXX_SRC})

target_link_libraries(fusion_cxx Mosek::Mosek)
target_include_directories(fusion_cxx PUBLIC ${CMAKE_CURRENT_LIST_DIR})
