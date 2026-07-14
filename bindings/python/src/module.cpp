// bindings/python/module — the nanobind module `quevedomp._native` (Task 4a.1, ADR-016).
//
// Each Phase 4a task adds one bind_*.cpp TU and calls its registrar here (Tasks 4a.2–4a.5).
// Binding rules (ADR-016): verb-level calls only — no Python callback is ever invoked from
// inside a C++ loop; blocking calls release the GIL; the core library is never modified for
// Python's benefit.
#include <nanobind/nanobind.h>

namespace nb = nanobind;

void bind_types(nb::module_ &m);     // Task 4a.2
void bind_robot(nb::module_ &m);     // Task 4a.3
void bind_collision(nb::module_ &m); // Task 4a.4
void bind_planning(nb::module_ &m);  // Task 4a.5
void bind_capture(nb::module_ &m);   // Task 4a.6 (Task 2a.5 serializers for studio save/load)
void bind_parameterization(nb::module_ &m); // Task 3.4 (PathSpline + time parameterization)

NB_MODULE(_native, m) {
  m.doc() = "QuevedoMP — GPU-accelerated robot-arm motion planning (nanobind bindings)";
  m.attr("__version__") = "0.1.0.dev0";

  bind_types(m);
  bind_robot(m);
  bind_collision(m);
  bind_planning(m);
  bind_capture(m);
  bind_parameterization(m);
}
