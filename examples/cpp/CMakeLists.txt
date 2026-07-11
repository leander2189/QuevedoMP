# Example executables. Build against the installed library; they use the viz API, which is a
# no-op unless the build set WITH_RERUN=ON (the dev-viz preset).
add_executable(quevedomp_visualize visualize.cpp)
target_link_libraries(quevedomp_visualize PRIVATE quevedomp::quevedomp)

add_executable(quevedomp_collision_visualize collision_visualize.cpp)
target_link_libraries(quevedomp_collision_visualize PRIVATE quevedomp::quevedomp)

# Full Phase 3a pipeline (IK -> RRT-Connect -> shortcut smooth) on the DTC inlet cell, animated in
# rerun. Uses FCL (or OptiX if built); .rrd only in a WITH_RERUN build. Reuses the DTC scene builder.
add_executable(inlet_plan_visualize inlet_plan_visualize.cpp)
target_link_libraries(inlet_plan_visualize PRIVATE quevedomp::quevedomp)
target_include_directories(inlet_plan_visualize PRIVATE ${CMAKE_SOURCE_DIR}/tests/support)
target_compile_definitions(inlet_plan_visualize
  PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures")

# Simple IK snapshot to SEE where the jointA EE collides with the inlet part (several IK branches
# as frames; EE mesh tinted by a real EE-vs-inlet check).
add_executable(inlet_ik_visualize inlet_ik_visualize.cpp)
target_link_libraries(inlet_ik_visualize PRIVATE quevedomp::quevedomp)
target_include_directories(inlet_ik_visualize PRIVATE ${CMAKE_SOURCE_DIR}/tests/support)
target_compile_definitions(inlet_ik_visualize
  PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures")

# DTC-cell OptiX-result visualization (Phase D). Needs the OptiX backend to classify poses; produces
# a .rrd only in a WITH_RERUN build (the viz-optix preset turns both on). Reuses the DTC scene builder.
if(QUEVEDOMP_WITH_OPTIX)
  add_executable(dtc_visualize dtc_visualize.cpp)
  target_link_libraries(dtc_visualize PRIVATE quevedomp::quevedomp)
  target_include_directories(dtc_visualize PRIVATE ${CMAKE_SOURCE_DIR}/tests/support)
  target_compile_definitions(dtc_visualize
    PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures")
endif()
