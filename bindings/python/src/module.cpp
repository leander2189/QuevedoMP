// bindings/python/module — the nanobind module `quevedomp._native` (Task 4a.1, ADR-016).
//
// Each Phase 4a task adds one bind_*.cpp TU and calls its registrar here (Tasks 4a.2–4a.5).
// Binding rules (ADR-016): verb-level calls only — no Python callback is ever invoked from
// inside a C++ loop; blocking calls release the GIL; the core library is never modified for
// Python's benefit.
#include <nanobind/nanobind.h>

namespace nb = nanobind;

void bind_types(nb::module_ &m); // Task 4a.2
void bind_robot(nb::module_ &m); // Task 4a.3

NB_MODULE(_native, m) {
  m.doc() = "QuevedoMP — GPU-accelerated robot-arm motion planning (nanobind bindings)";
  m.attr("__version__") = "0.1.0.dev0";

  bind_types(m);
  bind_robot(m);
}
