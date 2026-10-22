#include <cassert>

#include <humanoid_wb_id_mpc/constraint/WbIdTorqueWindow.h>

int main() {
  using ocs2::humanoid::WbIdTorqueWindow;
  WbIdTorqueWindow window(0.05);
  const ocs2::scalar_array_t times{1.0, 1.02, 1.04, 1.06, 1.08};
  window.setHorizonStart(2.0);  // Solver has advanced; the active policy has not.
  assert(window.containsInterpolatedInput(1.0, times));
  assert(window.containsInterpolatedInput(1.03, times));
  assert(window.containsInterpolatedInput(1.04, times));
  assert(!window.containsInterpolatedInput(1.045, times));
  assert(!window.containsInterpolatedInput(1.05, times));
  assert(!window.containsInterpolatedInput(1.07, times));
  assert(!window.containsInterpolatedInput(1.1, times));
  assert(!window.containsInterpolatedInput(1.0, {}));
  assert(window.containsInterpolatedInput(1.0, {1.0}));

  const ocs2::scalar_array_t events{1.0, 1.015, 1.015, 1.035, 1.055};
  assert(window.containsInterpolatedInput(1.015, events));
  assert(window.containsInterpolatedInput(1.025, events));
  assert(!window.containsInterpolatedInput(1.04, events));

  WbIdTorqueWindow full(-1.0);
  assert(full.containsInterpolatedInput(1.07, times));
  assert(!full.containsInterpolatedInput(1.1, times));
}
