// libFuzzer harness (Task 1.9): feed arbitrary bytes to RobotModel::from_urdf. The parser must
// never crash or trip ASan/UBSan on malformed input — it must either parse or throw. Build via
// the `fuzz` preset (clang + -fsanitize=fuzzer,address,undefined).
#include <cstddef>
#include <cstdint>
#include <exception>
#include <string>

#include "quevedomp/robot/robot_model.hpp"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
  try {
    const auto model =
        quevedomp::RobotModel::from_urdf(std::string(reinterpret_cast<const char *>(data), size));
    // Exercise downstream accessors on whatever parsed, so they are fuzzed too.
    (void)model->dof();
    (void)model->num_links();
    (void)model->num_joints();
    if (!model->links().empty()) {
      (void)model->chain_to(model->links().back().name);
    }
  } catch (const std::exception &) {
    // Throwing on bad input is the contract, not a bug.
  }
  return 0;
}
