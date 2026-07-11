# bindings/python — nanobind module `quevedomp._native` (Task 4a.1, ADR-016).
#
# Optional target behind QUEVEDOMP_WITH_PYTHON (OFF by default): the core library and every
# other target build bit-identical without it. nanobind is pulled by FetchContent at configure
# time (pinned tag, same pattern as the rerun SDK); Python dev headers come from the container
# (python3-dev). Build with the dev-py preset — sanitizers must stay OFF: an ASan-instrumented
# extension cannot load into an uninstrumented python interpreter.

find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development.Module)

include(FetchContent)
FetchContent_Declare(nanobind
  GIT_REPOSITORY https://github.com/wjakob/nanobind
  GIT_TAG        v2.4.0
  GIT_SHALLOW    ON)
FetchContent_MakeAvailable(nanobind)

# The extension is laid out as an importable package in the build tree:
#   <binaryDir>/bindings/python/quevedomp/{__init__.py, py.typed, _native.*.so}
# so `PYTHONPATH=<binaryDir>/bindings/python python3 -c "import quevedomp"` just works.
nanobind_add_module(quevedomp_native
  NB_STATIC
  src/module.cpp
)
target_link_libraries(quevedomp_native PRIVATE quevedomp::quevedomp)
set_target_properties(quevedomp_native PROPERTIES
  OUTPUT_NAME _native
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/quevedomp)

# Pure-Python package files, copied next to the extension (re-copied when they change).
foreach(f IN ITEMS quevedomp/__init__.py quevedomp/py.typed)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${f} ${CMAKE_CURRENT_BINARY_DIR}/${f} COPYONLY)
endforeach()

# Task 4a.1 verify gate: the module imports. pytest suites arrive with Tasks 4a.2+.
if(QUEVEDOMP_BUILD_TESTS)
  add_test(NAME python_import
    COMMAND Python::Interpreter -c "import quevedomp; print(quevedomp.__version__)")
  set_tests_properties(python_import PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}")
endif()
