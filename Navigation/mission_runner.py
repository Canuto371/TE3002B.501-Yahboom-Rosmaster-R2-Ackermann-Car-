import os
import signal
import subprocess
import sys
import time


class MissionRunner:
    def __init__(self):
        self.processes = []

        self.common_env = os.environ.copy()
        self.common_env["PYTHONUNBUFFERED"] = "1"

        # =============================
        # Parámetros de la misión
        # =============================

        self.initial_x = 0.4
        self.initial_y = 0.4
        self.initial_yaw = 0.0

        self.inflation_radius = 0.25

        self.follower_params = {
            "max_linear": 0.15,
            "min_linear": 0.05,
            "max_angular": 0.45,
            "lookahead_distance": 0.16,
            "final_goal_tolerance": 0.08,
            "angular_bias": 0.05,
            "pose_timeout": 3.0,
        }

        self.reverse_params = {
            "reverse_speed": 0.25,
            "angular_z": -0.6,
            "duration": 5.0,
        }

        self.goals = [
            (2.0, 2.0),
            (0.4, 2.0),
            (0.0, 0.0),
        ]

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
                env=self.common_env,
                preexec_fn=os.setsid
            )
        else:
            proc = subprocess.Popen(
                cmd,
                env=self.common_env,
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

    def wait_for_output(self, proc, name, success_patterns, fail_patterns=None, timeout_sec=180.0):
        if fail_patterns is None:
            fail_patterns = []

        start = time.time()

        while True:
            if time.time() - start > timeout_sec:
                print(f"[ERROR] Timeout waiting for {name}.")
                return False

            if proc.poll() is not None:
                print(f"[ERROR] {name} exited before success pattern.")
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
    # ROS COMMANDS
    # ==================================================

    def launch_localization(self):
        cmd = [
            "ros2", "launch", "navigation", "scan_match_localization.launch.py",
            f"initial_x:={self.initial_x}",
            f"initial_y:={self.initial_y}",
            f"initial_yaw:={self.initial_yaw}",
        ]

        proc = self.start_process(cmd, "localization", pipe_output=False)

        print("\nWaiting 10 seconds for base, LiDAR, map and scan matcher...")
        time.sleep(10.0)

        return proc

    def run_planner(self, goal_x, goal_y):
        cmd = [
            "ros2", "run", "navigation", "astar_planner_node",
            "--ros-args",
            "-p", f"goal_x:={goal_x}",
            "-p", f"goal_y:={goal_y}",
            "-p", f"inflation_radius:={self.inflation_radius}",
            "-p", "plan_once:=true",
        ]

        proc = self.start_process(
            cmd,
            f"planner_to_{goal_x}_{goal_y}",
            pipe_output=True
        )

        ok = self.wait_for_output(
            proc,
            f"planner_to_{goal_x}_{goal_y}",
            success_patterns=["Path published"],
            fail_patterns=[
                "No path found",
                "no nearby free cell found",
                "outside the map"
            ],
            timeout_sec=240.0
        )

        if not ok:
            self.stop_process(f"planner_to_{goal_x}_{goal_y}", proc)
            return None

        return proc

    def run_follower(self):
        cmd = [
            "ros2", "run", "navigation", "path_follower_node",
            "--ros-args",
        ]

        for key, value in self.follower_params.items():
            cmd += ["-p", f"{key}:={value}"]

        proc = self.start_process(" ".join(cmd) if False else cmd, "path_follower", pipe_output=True)

        ok = self.wait_for_output(
            proc,
            "path_follower",
            success_patterns=["FINAL GOAL REACHED"],
            fail_patterns=[],
            timeout_sec=240.0
        )

        self.stop_process("path_follower", proc)

        return ok

    def run_reverse_escape(self):
        cmd = [
            "ros2", "run", "navigation", "reverse_escape_node",
            "--ros-args",
            "-p", f"reverse_speed:={self.reverse_params['reverse_speed']}",
            "-p", f"angular_z:={self.reverse_params['angular_z']}",
            "-p", f"duration:={self.reverse_params['duration']}",
        ]

        proc = self.start_process(cmd, "reverse_escape", pipe_output=True)

        ok = self.wait_for_output(
            proc,
            "reverse_escape",
            success_patterns=["Reverse escape complete"],
            fail_patterns=[],
            timeout_sec=self.reverse_params["duration"] + 20.0
        )

        self.stop_process("reverse_escape", proc)

        return ok

    # ==================================================
    # MISSION
    # ==================================================

    def run_path_to_goal(self, goal_x, goal_y):
        planner_proc = self.run_planner(goal_x, goal_y)

        if planner_proc is None:
            print(f"[ERROR] Planner failed for goal ({goal_x}, {goal_y}).")
            return False

        print("\nPlanner is ready. Starting follower...")
        ok = self.run_follower()

        self.stop_process(f"planner_to_{goal_x}_{goal_y}", planner_proc)

        if not ok:
            print(f"[ERROR] Follower failed for goal ({goal_x}, {goal_y}).")
            return False

        print(f"[OK] Goal reached: ({goal_x}, {goal_y})")
        time.sleep(1.0)
        return True

    def run(self):
        localization_proc = None

        try:
            localization_proc = self.launch_localization()

            # Goal 1
            if not self.run_path_to_goal(2.0, 2.0):
                return 1

            # Goal 2
            if not self.run_path_to_goal(0.4, 2.0):
                return 1

            # Reverse maneuver
            print("\n========== RUNNING REVERSE ESCAPE ==========")
            if not self.run_reverse_escape():
                return 1

            time.sleep(1.0)

            # Goal 3
            if not self.run_path_to_goal(0.0, 0.0):
                return 1

            print("\n========== MISSION COMPLETE ==========")
            return 0

        except KeyboardInterrupt:
            print("\nMission interrupted by user.")
            return 130

        finally:
            print("\n========== CLEANUP ==========")

            # Stop all processes except ones already stopped
            for name, proc in reversed(self.processes):
                self.stop_process(name, proc)

            print("Cleanup complete.")


def main(args=None):
    runner = MissionRunner()
    code = runner.run()
    sys.exit(code)


if __name__ == "__main__":
    main()
