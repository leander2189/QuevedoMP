# Example executables. Build against the installed library; they use the viz API, which is a
# no-op unless the build set WITH_RERUN=ON (the dev-viz preset).
add_executable(quevedomp_visualize visualize.cpp)
target_link_libraries(quevedomp_visualize PRIVATE quevedomp::quevedomp)

add_executable(quevedomp_collision_visualize collision_visualize.cpp)
target_link_libraries(quevedomp_collision_visualize PRIVATE quevedomp::quevedomp)

# DTC-cell OptiX-result visualization (Phase D). Needs the OptiX backend to classify poses; produces
# a .rrd only in a WITH_RERUN build (the viz-optix preset turns both on). Reuses the DTC scene builder.
if(QUEVEDOMP_WITH_OPTIX)
  add_executable(dtc_visualize dtc_visualize.cpp)
  target_link_libraries(dtc_visualize PRIVATE quevedomp::quevedomp)
  target_include_directories(dtc_visualize PRIVATE ${CMAKE_SOURCE_DIR}/tests/support)
  target_compile_definitions(dtc_visualize
    PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures")
endif()
