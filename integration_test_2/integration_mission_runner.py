import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


class IntegrationMissionRunner:
    def __init__(self, target):
        self.target = target.upper()

        if self.target not in ["A", "B", "C"]:
            raise ValueError("target must be A, B, or C")

        self.processes = []
        self.env = os.environ.copy()
        self.env["PYTHONUNBUFFERED"] = "1"

        self.pkg_path = Path("/root/vegas_ws/integration_test_2")
        self.config = json.loads((self.pkg_path / "config" / "checkpoints.json").read_text())

        self.checkpoints = self.config["checkpoints"]

        self.initial_x = 0.4
        self.initial_y = 0.4
        self.initial_yaw = 0.0

        self.normal_inflation = float(self.config["inflation"]["normal_astar"])
        self.after_reverse_inflation = float(self.config["inflation"]["after_reverse_astar"])

        # A* interno con grid más gruesa para acelerar planeación.
        self.planner_resolution = 0.04

        self.follower_params = {
            "max_linear": 0.16,
            "min_linear": 0.07,
            "max_angular": 0.45,
            "lookahead_distance": 0.16,
            "final_goal_tolerance": 0.08,
            "angular_bias": 0.05,
            "pose_timeout": 3.0,
            "cmd_vel_topic": "/cmd_vel_raw",
        }

    # ==================================================
    # PROCESS HELPERS
    # ==================================================

    def start_process(self, cmd, name, pipe_output=False):
        print(f"\n========== STARTING {name} ==========")
        print(" ".join(cmd))

        if pipe_output:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=self.env,
                preexec_fn=os.setsid
            )
        else:
            proc = subprocess.Popen(
                cmd,
                env=self.env,
                preexec_fn=os.setsid
            )

        self.processes.append((name, proc))
        return proc

    def stop_process(self, name, proc):
        if proc is None:
            return

        if proc.poll() is not None:
            return

        print(f"\n========== STOPPING {name} ==========")

        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=3.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass

    def wait_for_output(self, proc, name, success_patterns, fail_patterns=None, timeout_sec=300.0):
        if fail_patterns is None:
            fail_patterns = []

        start = time.time()

        while True:
            if time.time() - start > timeout_sec:
                print(f"[ERROR] Timeout waiting for {name}")
                return False

            if proc.poll() is not None:
                print(f"[ERROR] {name} exited before success pattern")
                return False

            line = proc.stdout.readline()

            if not line:
                time.sleep(0.02)
                continue

            line = line.rstrip()
            print(f"[{name}] {line}")

            for fail in fail_patterns:
                if fail in line:
                    print(f"[ERROR] {name} failed because pattern was found: {fail}")
                    return False

            for success in success_patterns:
                if success in line:
                    print(f"[OK] {name}: found success pattern: {success}")
                    return True

    # ==================================================
    # ACTIONS
    # ==================================================

    def launch_localization(self):
        cmd = [
            "ros2", "launch", "navigation", "scan_match_localization.launch.py",
            f"initial_x:={self.initial_x}",
            f"initial_y:={self.initial_y}",
            f"initial_yaw:={self.initial_yaw}",
        ]

        proc = self.start_process(cmd, "localization", pipe_output=False)

        print("\nWaiting 12 seconds for base, LiDAR, map and scan matcher...")
        time.sleep(12.0)

        return proc

    def launch_map_republisher(self):
        cmd = [
            "ros2", "run", "integration_test_2", "map_republisher_node",
            "--ros-args",
            "-p", "input_topic:=/map",
            "-p", "output_topic:=/map_for_planning",
            "-p", "publish_rate:=1.0",
        ]

        self.start_process(cmd, "map_republisher", pipe_output=False)

        print("\nWaiting 2 seconds for map republisher...")
        time.sleep(2.0)

    def launch_signal_system(self):
        # IMPORTANTE:
        # La misión NO arranca la visión.
        # sift_signal_node se corre en otra terminal y solo publica /detected_signal.
        # Aquí solo arrancamos el gate que modifica /cmd_vel_raw -> /cmd_vel.
        gate_cmd = [
            "ros2", "run", "integration_test_2", "signal_gate_node",
            "--ros-args",
            "-p", "manual_loading_confirmation:=false",
            "-p", "loading_wait_seconds:=5.0",
            "-p", "restricted_area_ratio_stop:=0.06",
            "-p", "pedestrian_speed_scale:=0.7",
            "-p", "stop_duration:=5.0",
        ]

        self.start_process(gate_cmd, "signal_gate", pipe_output=False)

        print("\nWaiting 1 second for signal gate...")
        time.sleep(1.0)

    def launch_signal_system(self):
        gate_cmd = [
            "ros2", "run", "integration_test_2", "signal_gate_node",
            "--ros-args",
            "-p", "manual_loading_confirmation:=false",
            "-p", "loading_wait_seconds:=5.0",
            "-p", "restricted_area_ratio_stop:=0.06",
            "-p", "pedestrian_speed_scale:=0.5",
            "-p", "stop_duration:=5.0",
        ]

        vision_cmd = [
            "ros2", "run", "integration_test_2", "sift_signal_node"
        ]

        self.start_process(gate_cmd, "signal_gate", pipe_output=False)
        self.start_process(vision_cmd, "sift_signal_node", pipe_output=False)

        print("\nWaiting 3 seconds for signal gate and vision node...")
        time.sleep(3.0)

    def run_astar_to_checkpoint(self, checkpoint_name, inflation_radius, label=None):
        x, y = self.checkpoints[checkpoint_name]

        if label is None:
            label = f"astar_to_{checkpoint_name}"

        return self.run_astar_to_point(
            name=label,
            goal_x=x,
            goal_y=y,
            inflation_radius=inflation_radius
        )

    def run_astar_to_point(self, name, goal_x, goal_y, inflation_radius):
        cmd = [
            "ros2", "run", "navigation", "astar_planner_node",
            "--ros-args",
            "-p", f"goal_x:={goal_x}",
            "-p", f"goal_y:={goal_y}",
            "-p", f"inflation_radius:={inflation_radius}",
            "-p", f"planner_resolution:={self.planner_resolution}",
            "-p", "map_topic:=/map_for_planning",
            "-p", "plan_once:=true",
        ]

        proc = self.start_process(cmd, name, pipe_output=True)

        ok = self.wait_for_output(
            proc,
            name,
            success_patterns=["Path published"],
            fail_patterns=[
                "No path found",
                "no nearby free cell found",
                "outside map",
                "outside the map"
            ],
            timeout_sec=300.0
        )

        if not ok:
            self.stop_process(name, proc)
            return None

        return proc

    def run_follower(self):
        cmd = [
            "ros2", "run", "navigation", "path_follower_node",
            "--ros-args",
        ]

        for key, value in self.follower_params.items():
            cmd += ["-p", f"{key}:={value}"]

        proc = self.start_process(cmd, "path_follower", pipe_output=True)

        ok = self.wait_for_output(
            proc,
            "path_follower",
            success_patterns=["FINAL GOAL REACHED"],
            timeout_sec=300.0
        )

        self.stop_process("path_follower", proc)

        return ok

    def plan_and_follow_checkpoint(self, checkpoint_name, inflation_radius, label=None):
        planner_name = label or f"astar_to_{checkpoint_name}"

        planner = self.run_astar_to_checkpoint(
            checkpoint_name=checkpoint_name,
            inflation_radius=inflation_radius,
            label=planner_name
        )

        if planner is None:
            return False

        print(f"\nPlanner ready for {checkpoint_name}. Starting follower...")
        ok = self.run_follower()

        self.stop_process(planner_name, planner)

        if ok:
            print(f"[OK] Reached checkpoint {checkpoint_name}")

        return ok

    def run_reverse(self):
        reverse = self.config["reverse_routines"][self.target]

        cmd = [
            "ros2", "run", "navigation", "reverse_escape_node",
            "--ros-args",
            "-p", f"reverse_speed:={reverse['reverse_speed']}",
            "-p", f"angular_z:={reverse['angular_z']}",
            "-p", f"duration:={reverse['duration']}",
        ]

        proc = self.start_process(cmd, f"reverse_{self.target}", pipe_output=True)

        ok = self.wait_for_output(
            proc,
            f"reverse_{self.target}",
            success_patterns=["Reverse escape complete"],
            timeout_sec=float(reverse["duration"]) + 20.0
        )

        self.stop_process(f"reverse_{self.target}", proc)

        return ok

    # ==================================================
    # MISSION
    # ==================================================

    def run(self):
        try:
            self.launch_localization()
            self.launch_map_republisher()
            self.launch_signal_system()

            print(f"\n========== TARGET SELECTED: {self.target} ==========")

            # 1. Current pose -> MIDIN
            if not self.plan_and_follow_checkpoint(
                checkpoint_name="MIDIN",
                inflation_radius=self.normal_inflation,
                label="astar_current_to_MIDIN"
            ):
                return 1

            # 2. MIDIN -> target A/B/C
            if not self.plan_and_follow_checkpoint(
                checkpoint_name=self.target,
                inflation_radius=self.normal_inflation,
                label=f"astar_MIDIN_to_{self.target}"
            ):
                return 1

            # 3. Simulated loading wait at target checkpoint
            print("\n========== LOADING WAIT AT TARGET ==========")
            print("Waiting 5 seconds before reverse...")
            time.sleep(5.0)

            # 4. Reverse routine based on target
            if not self.run_reverse():
                return 1

            # 4. Current pose after reverse -> START
            if not self.plan_and_follow_checkpoint(
                checkpoint_name="START",
                inflation_radius=self.normal_inflation,
                label="astar_after_reverse_to_START"
            ):
                return 1

            print("\n========== INTEGRATION MISSION COMPLETE ==========")
            return 0

        except KeyboardInterrupt:
            print("\nMission interrupted by user.")
            return 130

        finally:
            print("\n========== CLEANUP ==========")

            for name, proc in reversed(self.processes):
                self.stop_process(name, proc)

            print("Cleanup complete.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--target",
        default="B",
        choices=["A", "B", "C", "a", "b", "c"],
        help="Target selected by vision. Default: B"
    )

    args = parser.parse_args()

    runner = IntegrationMissionRunner(args.target)
    code = runner.run()
    sys.exit(code)


if __name__ == "__main__":
    main()
