#include <gtest/gtest.h>

// Phase 0 sanity: proves the toolchain compiles, links GTest, and runs ctest.
// Real unit tests arrive with library code in Phase 1.
TEST(Bootstrap, Sanity) { EXPECT_EQ(1 + 1, 2); }
