# Example executables. Build against the installed library; they use the viz API, which is a
# no-op unless the build set WITH_RERUN=ON (the dev-viz preset).
add_executable(quevedomp_visualize visualize.cpp)
target_link_libraries(quevedomp_visualize PRIVATE quevedomp::quevedomp)
