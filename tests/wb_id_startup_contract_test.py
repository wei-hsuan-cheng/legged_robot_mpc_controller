#!/usr/bin/env python3
"""Static WB-ID startup/thread-ownership regression; no ROS build required."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src/humanoid_whole_body_inverse_dynamics_mpc/"
          "humanoid_whole_body_inverse_dynamics_mpc_controller.cpp").read_text()


def body(name):
    return SOURCE.split(f"HumanoidWholeBodyInverseDynamicsMpcController::{name}(", 1)[1].split("\n}\n", 1)[0]


class StartupContractTest(unittest.TestCase):
    def test_observations_arrive_before_policy_and_phase_checks(self):
        update = body("update_and_write_commands")
        observation = update.index("mrt_interface_->setCurrentObservation(observation)")
        self.assertLess(observation, update.index("mrt_interface_->updatePolicy()"))
        self.assertLess(observation, update.index("run_bring_up(time, observation)"))
        self.assertNotIn("setCurrentObservation", body("compute_mpc_joint_action"))

    def test_first_solve_waits_for_a_post_construction_observation(self):
        start = body("start_deferred_solver_thread")
        self.assertLess(start.index("construct_solver(start_observation_)"), start.index("mpc_ready_.store(true"))
        self.assertLess(start.index("mpc_ready_.store(true"), start.index("solver_observation_ready_.load"))
        self.assertLess(start.index("solver_observation_ready_.load"), start.index("solver_worker()"))
        update = body("update_and_write_commands")
        self.assertLess(update.index("setCurrentObservation(observation)"),
                        update.index("solver_observation_ready_.store(true"))

    def test_expired_and_future_policies_are_rejected(self):
        update = body("update_and_write_commands")
        self.assertIn("evaluation_time >= times.front()", update)
        self.assertIn("evaluation_time + margin < times.back()", update)
        self.assertIn("times.size() >= 2", update)
        self.assertIn("std::isfinite(times.front())", update)
        self.assertIn("std::isfinite(times.back())", update)
        self.assertIn("if (policy_updated || previous_policy_ready)", update)

    def test_solver_reset_is_worker_owned(self):
        update = body("update_and_write_commands")
        worker = body("solver_worker")
        self.assertNotIn("resetMpcNode", update)
        self.assertIn("reset_mpc_warm_start_requested_.store(true", update)
        reset = worker.index("mrt_interface_->resetMpcNode(")
        self.assertLess(worker.index("reset_mpc_warm_start_requested_.exchange(false"), reset)
        self.assertLess(reset, worker.index("mrt_interface_->advanceMpc()"))
        self.assertNotIn("mrt_interface_->updatePolicy()", worker)
        self.assertNotIn("mrt_interface_->reset()", worker)

    def test_bring_up_does_not_evaluate_an_unusable_policy(self):
        bring_up = body("run_bring_up")
        starting = bring_up.split("case BringUpPhase::StartingMpc:", 1)[1].split("case BringUpPhase::Blend:", 1)[0]
        self.assertIn("if (active_policy_ready_)", starting)
        blend = bring_up.split("case BringUpPhase::Blend:", 1)[1].split("case BringUpPhase::Locomotion:", 1)[0]
        self.assertLess(blend.index("if (!active_policy_ready_)"), blend.index("compute_mpc_joint_action(observation)"))
        self.assertIn("enter_bring_up_phase(BringUpPhase::StartingMpc, t)", blend)
        locomotion = bring_up.split("case BringUpPhase::Locomotion:", 1)[1].split("case BringUpPhase::Fault:", 1)[0]
        self.assertLess(locomotion.index("if (!active_policy_ready_)"), locomotion.index("compute_mpc_joint_action(observation)"))
        self.assertIn("enter_bring_up_phase(BringUpPhase::Fault, t)", locomotion)

    def test_activation_clears_readiness_and_reset_requests(self):
        activate = body("on_activate")
        for reset in ("solver_observation_ready_.store(false)", "reset_mpc_warm_start_requested_.store(false)",
                      "has_active_policy_ = false", "active_policy_ready_ = false"):
            self.assertIn(reset, activate)


if __name__ == "__main__":
    unittest.main()
