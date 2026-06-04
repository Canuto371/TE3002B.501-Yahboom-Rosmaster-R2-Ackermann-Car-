import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory


class MissionOrchestrator(Node):
    def __init__(self):
        super().__init__('mission_orchestrator_node')

        self.declare_parameter('target', 'B')
        self.declare_parameter('config_file', '')

        self.target = str(self.get_parameter('target').value).upper()

        if self.target not in ['A', 'B', 'C']:
            raise ValueError('target must be A, B, or C')

        config_file = str(self.get_parameter('config_file').value)

        if config_file:
            self.config_path = Path(config_file)
        else:
            pkg_share = Path(get_package_share_directory('integration_test_4'))
            self.config_path = pkg_share / 'config' / 'mission_config.json'

        self.config = json.loads(self.config_path.read_text())

        self.checkpoints = self.config['checkpoints']
        self.astar_cfg = self.config['astar']
        self.follower_cfg = self.config['follower']
        self.reverse_cfg = self.config['reverse_routines']
        self.mission_cfg = self.config['mission']

        self.processes = []
        self.env = os.environ.copy()
        self.env['PYTHONUNBUFFERED'] = '1'

        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        self.event_pub = self.create_publisher(String, '/mission_event', 10)

        self.get_logger().info(
            f'Mission orchestrator ready | target={self.target} | config={self.config_path}'
        )

    # ==================================================
    # ROS STATE / EVENT
    # ==================================================

    def publish_state(self, state):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.get_logger().info(f'MISSION_STATE: {state}')

    def publish_event(self, event):
        msg = String()
        msg.data = event
        self.event_pub.publish(msg)
        self.get_logger().info(f'MISSION_EVENT: {event}')

    # ==================================================
    # PROCESS HELPERS
    # ==================================================

    def start_process(self, cmd, name, pipe_output=False):
        self.get_logger().info(f'STARTING {name}: {" ".join(cmd)}')

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

        self.get_logger().info(f'STOPPING {name}')

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
                self.get_logger().error(f'Timeout waiting for {name}')
                return False

            if proc.poll() is not None:
                self.get_logger().error(f'{name} exited before success pattern')
                return False

            line = proc.stdout.readline()

            if not line:
                time.sleep(0.02)
                continue

            line = line.rstrip()
            print(f'[{name}] {line}', flush=True)

            for fail in fail_patterns:
                if fail in line:
                    self.get_logger().error(
                        f'{name} failed because pattern was found: {fail}'
                    )
                    return False

            for success in success_patterns:
                if success in line:
                    self.get_logger().info(
                        f'{name}: success pattern found: {success}'
                    )
                    return True

    # ==================================================
    # SYSTEM STARTUP
    # ==================================================

    def launch_localization(self):
        initial_x = self.mission_cfg['initial_x']
        initial_y = self.mission_cfg['initial_y']
        initial_yaw = self.mission_cfg['initial_yaw']

        cmd = [
            'ros2', 'launch', 'navigation', 'scan_match_localization.launch.py',
            f'initial_x:={initial_x}',
            f'initial_y:={initial_y}',
            f'initial_yaw:={initial_yaw}',
        ]

        proc = self.start_process(cmd, 'localization', pipe_output=False)

        wait_time = float(self.mission_cfg['localization_startup_wait'])
        self.get_logger().info(f'Waiting {wait_time:.1f}s for localization startup...')
        time.sleep(wait_time)

        return proc

    def launch_map_republisher(self):
        cmd = [
            'ros2', 'run', 'integration_test_4', 'map_republisher_node',
            '--ros-args',
            '-p', 'input_topic:=/map',
            '-p', 'output_topic:=/map_for_planning',
            '-p', 'publish_rate:=1.0',
        ]

        proc = self.start_process(cmd, 'map_republisher', pipe_output=False)

        wait_time = float(self.mission_cfg['map_republisher_startup_wait'])
        self.get_logger().info(f'Waiting {wait_time:.1f}s for map republisher startup...')
        time.sleep(wait_time)

        return proc

    def launch_signal_gate(self):
        cmd = [
            'ros2', 'run', 'integration_test_4', 'signal_gate_node'
        ]

        proc = self.start_process(cmd, 'signal_gate', pipe_output=False)

        self.get_logger().info('Waiting 1.0s for signal gate startup...')
        time.sleep(1.0)

        return proc

    # ==================================================
    # ACTIONS
    # ==================================================

    def run_astar_to_checkpoint(self, checkpoint_name, label=None):
        x, y = self.checkpoints[checkpoint_name]

        planner_name = label or f'astar_to_{checkpoint_name}'

        cmd = [
            'ros2', 'run', 'navigation', 'astar_planner_node',
            '--ros-args',
            '-p', f'goal_x:={x}',
            '-p', f'goal_y:={y}',
            '-p', f'inflation_radius:={self.astar_cfg["inflation_radius"]}',
            '-p', f'planner_resolution:={self.astar_cfg["planner_resolution"]}',
            '-p', f'map_topic:={self.astar_cfg["map_topic"]}',
            '-p', 'plan_once:=true',
        ]

        proc = self.start_process(cmd, planner_name, pipe_output=True)

        ok = self.wait_for_output(
            proc,
            planner_name,
            success_patterns=['Path published'],
            fail_patterns=[
                'No path found',
                'no nearby free cell found',
                'outside map',
                'outside the map'
            ],
            timeout_sec=float(self.mission_cfg['planner_timeout'])
        )

        if not ok:
            self.stop_process(planner_name, proc)
            return None

        return proc

    def run_follower(self):
        cmd = [
            'ros2', 'run', 'navigation', 'lyapunov_path_follower_node',
            '--ros-args',
        ]

        for key, value in self.follower_cfg.items():
            cmd += ['-p', f'{key}:={value}']

        proc = self.start_process(cmd, 'lyapunov_path_follower', pipe_output=True)

        ok = self.wait_for_output(
            proc,
            'lyapunov_path_follower',
            success_patterns=['FINAL GOAL REACHED'],
            timeout_sec=float(self.mission_cfg['follower_timeout'])
        )

        self.stop_process('lyapunov_path_follower', proc)

        return ok


    def plan_and_follow(self, checkpoint_name, state_name, planner_label):
        self.publish_state(state_name)

        planner = self.run_astar_to_checkpoint(
            checkpoint_name=checkpoint_name,
            label=planner_label
        )

        if planner is None:
            self.publish_event(f'planner_failed:{checkpoint_name}')
            return False

        self.publish_event(f'path_ready:{checkpoint_name}')

        ok = self.run_follower()

        self.stop_process(planner_label, planner)

        if not ok:
            self.publish_event(f'follower_failed:{checkpoint_name}')
            return False

        self.publish_event(f'reached:{checkpoint_name}')
        return True

    def wait_at_target(self):
        wait_time = float(self.mission_cfg['target_wait_seconds'])

        self.publish_state('WAITING_AT_TARGET')
        self.publish_event(f'target_wait_started:{wait_time:.1f}')

        time.sleep(wait_time)

        self.publish_event('target_wait_complete')

    def run_reverse(self):
        cfg = self.reverse_cfg[self.target]

        cmd = [
            'ros2', 'run', 'navigation', 'reverse_escape_node',
            '--ros-args',
            '-p', f'reverse_speed:={cfg["reverse_speed"]}',
            '-p', f'angular_z:={cfg["angular_z"]}',
            '-p', f'duration:={cfg["duration"]}',
        ]

        name = f'reverse_{self.target}'

        self.publish_state('REVERSING')
        self.publish_event(f'reverse_started:{self.target}')

        proc = self.start_process(cmd, name, pipe_output=True)

        ok = self.wait_for_output(
            proc,
            name,
            success_patterns=['Reverse escape complete'],
            timeout_sec=float(cfg['duration']) + 20.0
        )

        self.stop_process(name, proc)

        if ok:
            self.publish_event(f'reverse_complete:{self.target}')
        else:
            self.publish_event(f'reverse_failed:{self.target}')

        return ok

    # ==================================================
    # MISSION
    # ==================================================

    def run_mission(self):
        try:
            self.publish_state('STARTING_SYSTEM')

            self.launch_localization()
            self.launch_map_republisher()
            self.launch_signal_gate()

            self.publish_event(f'target_selected:{self.target}')

            # 1. Current pose -> MIDIN
            if not self.plan_and_follow(
                checkpoint_name='MIDIN',
                state_name='GOING_TO_MIDIN',
                planner_label='astar_current_to_MIDIN'
            ):
                return 1

            # 2. Current pose -> target A/B/C
            if not self.plan_and_follow(
                checkpoint_name=self.target,
                state_name=f'GOING_TO_TARGET_{self.target}',
                planner_label=f'astar_MIDIN_to_{self.target}'
            ):
                return 1

            # 3. Wait at target
            self.wait_at_target()

            # 4. Reverse
            if not self.run_reverse():
                return 1

            # 5. Current pose -> START
            if not self.plan_and_follow(
                checkpoint_name='START',
                state_name='RETURNING_TO_START',
                planner_label='astar_after_reverse_to_START'
            ):
                return 1

            self.publish_state('MISSION_COMPLETE')
            self.publish_event('mission_complete')

            return 0

        except KeyboardInterrupt:
            self.publish_event('mission_interrupted')
            return 130

        finally:
            self.cleanup()

    def cleanup(self):
        self.publish_state('CLEANUP')

        for name, proc in reversed(self.processes):
            self.stop_process(name, proc)

        self.publish_event('cleanup_complete')


def main(args=None):
    rclpy.init(args=args)

    node = MissionOrchestrator()

    code = node.run_mission()

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(code)


if __name__ == '__main__':
    main()
