add_library(chugjug_lib STATIC)

target_sources(chugjug_lib PRIVATE
    src/lis3dh.cpp
)

target_include_directories(chugjug_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(chugjug_lib PUBLIC
    stm32cubemx
)
