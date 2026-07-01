// Task 2b.1 (scaffolding) — the OptiX toolchain works end-to-end on the GPU: PTX module ->
// pipeline -> SBT -> launch, with results copied back. Built + run only under the dev-optix preset
// (QUEVEDOMP_WITH_OPTIX). As tracing lands this file grows the FCL-agreement boolean tests.
#include <gtest/gtest.h>

#include <string>

namespace quevedomp::collision {
bool optix_selftest(std::string &err);
}

TEST(OptixBackend, TracesEnvironmentGas) {
  std::string err;
  EXPECT_TRUE(quevedomp::collision::optix_selftest(err)) << err;
}
