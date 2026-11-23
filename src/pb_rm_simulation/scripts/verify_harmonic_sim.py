#!/usr/bin/env python3
"""Headless end-to-end verification suite for the Gazebo Harmonic port of pb_rm_simulation.

Verifies:
  - Simulation real-time factor (RTF >= 0.85) under OGRE2 GPU rendering
  - 3D ground-truth odometry (/sim/ground_truth/odom @ ~50 Hz, zero /tf conflict with LIO)
  - IMU (/livox/imu @ ~200 Hz, frame_id=imu_link, acceleration in m/s^2)
  - Wheel joint states (/joint_states @ ~50 Hz)
  - OmniChassisSystem static hold drift and holonomic drive tracking (/cmd_vel)
  - Livox Mid-360 GPU lidar (/livox/lidar CustomMsg + 26-byte LivoxPointXyzrtlt PointCloud2)
  - Optional STL raycast interior accuracy & noise stddev check (RMUC2027)
  - Optional 5-subframe yaw motion deskew alignment against IMU angular velocity
  - Optional dynamic obstacles motion + <laser_retro> GPU lidar detection (RMUL / RMUL2026H)
  - Zero Gazebo/ROS error lines in launch logs
"""

import argparse
import math
import os
import signal
import struct
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, PointCloud2
from tf2_msgs.msg import TFMessage


LIVOX_PC2_DTYPE = np.dtype([
    ('x', '<f4'),
    ('y', '<f4'),
    ('z', '<f4'),
    ('intensity', '<f4'),
    ('tag', 'u1'),
    ('line', 'u1'),
    ('timestamp', '<f8'),
])
assert LIVOX_PC2_DTYPE.itemsize == 26


class HarmonicSimVerifier(Node):
    def __init__(self):
        super().__init__('harmonic_sim_verifier')
        self.clock_samples = []
        self.imu_msgs = []
        self.joint_msgs = []
        self.odom_msgs = []
        self.custom_msgs = []
        self.cloud2_msgs = []
        self.tf_pairs = set()

        self.create_subscription(Clock, '/clock', self._on_clock, 50)
        self.create_subscription(Imu, '/livox/imu', self._on_imu, 500)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 100)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self._on_odom, 100)
        self.create_subscription(CustomMsg, '/livox/lidar', self._on_custom, 30)
        self.create_subscription(PointCloud2, '/livox/lidar/pointcloud', self._on_cloud2, 30)
        self.create_subscription(TFMessage, '/tf', self._on_tf, 100)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _on_clock(self, msg):
        self.clock_samples.append((time.monotonic(), msg.clock.sec + msg.clock.nanosec * 1e-9))

    def _on_imu(self, msg):
        self.imu_msgs.append(msg)

    def _on_joints(self, msg):
        self.joint_msgs.append(msg)

    def _on_odom(self, msg):
        self.odom_msgs.append(msg)

    def _on_custom(self, msg):
        self.custom_msgs.append((self.latest_sim_time(), msg))

    def _on_cloud2(self, msg):
        self.cloud2_msgs.append(msg)

    def _on_tf(self, msg):
        for tr in msg.transforms:
            self.tf_pairs.add((tr.header.frame_id, tr.child_frame_id))

    def latest_sim_time(self):
        return self.clock_samples[-1][1] if self.clock_samples else 0.0

    def send_cmd(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)


def drain(node):
    while True:
        prev = (
            len(node.clock_samples)
            + len(node.imu_msgs)
            + len(node.joint_msgs)
            + len(node.odom_msgs)
            + len(node.custom_msgs)
            + len(node.cloud2_msgs)
        )
        rclpy.spin_once(node, timeout_sec=0.0)
        cur = (
            len(node.clock_samples)
            + len(node.imu_msgs)
            + len(node.joint_msgs)
            + len(node.odom_msgs)
            + len(node.custom_msgs)
            + len(node.cloud2_msgs)
        )
        if cur == prev:
            break


def spin_for_sim(node, sim_duration, cmd=None):
    drain(node)
    start_sim = node.latest_sim_time()
    last_cmd_wall = 0.0
    deadline = time.monotonic() + max(25.0, sim_duration * 6.0)
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.005)
        now = time.monotonic()
        if cmd is not None and (now - last_cmd_wall) >= 0.02:
            node.send_cmd(*cmd)
            last_cmd_wall = now
        if node.latest_sim_time() - start_sim >= sim_duration:
            return True
    return False


def get_gz_model_poses(names):
    out = subprocess.check_output(
        ['gz', 'topic', '-e', '-t', '/world/default/pose/info', '-n', '1'],
        text=True,
        timeout=5,
    )
    results = {}
    target_set = set(names)
    for blk in out.split('pose {')[1:]:
        matched = None
        for line in blk.splitlines():
            s = line.strip()
            if s.startswith('name: "'):
                nm = s.split('"')[1]
                if nm in target_set:
                    matched = nm
                break
        if matched is None:
            continue
        pos = [0.0, 0.0, 0.0]
        ori = [0.0, 0.0, 0.0, 1.0]
        in_pos = in_ori = False
        for line in blk.splitlines():
            s = line.strip()
            if s.startswith('position {'):
                in_pos, in_ori = True, False
            elif s.startswith('orientation {'):
                in_pos, in_ori = False, True
            elif s == '}':
                in_pos = in_ori = False
            elif in_pos:
                if s.startswith('x:'):
                    pos[0] = float(s.split(':')[1])
                elif s.startswith('y:'):
                    pos[1] = float(s.split(':')[1])
                elif s.startswith('z:'):
                    pos[2] = float(s.split(':')[1])
            elif in_ori:
                if s.startswith('x:'):
                    ori[0] = float(s.split(':')[1])
                elif s.startswith('y:'):
                    ori[1] = float(s.split(':')[1])
                elif s.startswith('z:'):
                    ori[2] = float(s.split(':')[1])
                elif s.startswith('w:'):
                    ori[3] = float(s.split(':')[1])
        results[matched] = (np.array(pos, dtype=np.float64), np.array(ori, dtype=np.float64))
    return results


def quat_to_rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def load_stl_triangles(stl_path, offset):
    with open(stl_path, 'rb') as f:
        f.read(80)
        ntri = struct.unpack('<I', f.read(4))[0]
        dt = np.dtype([
            ('n', '<f4', 3),
            ('v0', '<f4', 3),
            ('v1', '<f4', 3),
            ('v2', '<f4', 3),
            ('attr', '<u2'),
        ])
        data = np.frombuffer(f.read(ntri * 50), dtype=dt)
    v0 = data['v0'].astype(np.float64) + offset
    e1 = data['v1'].astype(np.float64) + offset - v0
    e2 = data['v2'].astype(np.float64) + offset - v0
    return v0, e1, e2


def raycast_stl(origin, ray_dir, v0, e1, e2):
    h = np.cross(ray_dir, e2)
    a = np.einsum('ij,ij->i', e1, h)
    valid = np.abs(a) > 1e-9
    f = np.zeros_like(a)
    f[valid] = 1.0 / a[valid]
    s = origin - v0
    u = f * np.einsum('ij,ij->i', s, h)
    valid &= (u >= 0.0) & (u <= 1.0)
    q = np.cross(s[valid], e1[valid])
    v = f[valid] * np.dot(q, ray_dir)
    valid_idx = np.where(valid)[0]
    m2 = (v >= 0.0) & (u[valid] + v <= 1.0)
    if not np.any(m2):
        return math.inf
    idx2 = valid_idx[m2]
    t = f[idx2] * np.einsum('ij,ij->i', e2[idx2], q[m2])
    t = t[t >= 0.1]
    return float(np.min(t)) if len(t) > 0 else math.inf


def run_case(
    world='RMUC2027',
    noise_std=0.02,
    dynamic_obstacles=False,
    check_geometry=False,
    check_motion=False,
    stl_path='src/pb_rm_simulation/meshes/RMUC2027_world/meshes/rmuc_2027.stl',
):
    tag = f'{world}{"_dyn" if dynamic_obstacles else ""}_noise{noise_std}'
    log_dir = tempfile.gettempdir()
    log_path = os.path.join(log_dir, f'verify_harmonic_{tag}.log')
    cmd = [
        'ros2', 'launch', 'pb_rm_simulation', 'rm_simulation.launch.py',
        f'world:={world}',
        f'dynamic_obstacles:={"true" if dynamic_obstacles else "false"}',
        'gui:=false',
        'rviz:=false',
        'use_livox:=true',
        f'lidar_noise_stddev:={noise_std}',
    ]
    log_f = open(log_path, 'w')
    proc = subprocess.Popen(cmd, stdout=log_f, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

    rclpy.init()
    node = HarmonicSimVerifier()
    try:
        # Wait for robot to settle and topics to flow (~2.0s sim time)
        deadline = time.monotonic() + 35.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if (
                len(node.custom_msgs) >= 5
                and len(node.odom_msgs) >= 20
                and node.latest_sim_time() >= 2.0
            ):
                break
        assert len(node.custom_msgs) >= 5, f'No /livox/lidar messages received on {tag}!'
        assert len(node.odom_msgs) >= 20, f'No /sim/ground_truth/odom messages received on {tag}!'

        dyn_names = ['dynamic_obstacle_1', 'dynamic_obstacle_2'] if dynamic_obstacles else []
        gz_poses_t0 = get_gz_model_poses(['robot'] + dyn_names) if dynamic_obstacles else None

        # Static 3.0s measurement window
        c0 = len(node.clock_samples)
        i0 = len(node.imu_msgs)
        j0 = len(node.joint_msgs)
        o0 = len(node.odom_msgs)
        l0 = len(node.custom_msgs)
        p0 = len(node.cloud2_msgs)
        assert spin_for_sim(node, 3.0), f'Simulation stalled during 3s window on {tag}!'

        w0, s0 = node.clock_samples[c0]
        w1, s1 = node.clock_samples[-1]
        d_sim = s1 - s0
        d_wall = w1 - w0
        rtf = d_sim / d_wall if d_wall > 0 else 0.0

        window_imu = node.imu_msgs[i0:]
        window_joints = node.joint_msgs[j0:]
        window_odom = node.odom_msgs[o0:]
        window_custom = node.custom_msgs[l0:]
        window_cloud2 = node.cloud2_msgs[p0:]

        imu_hz = len(window_imu) / d_sim
        joint_hz = len(window_joints) / d_sim
        odom_hz = len(window_odom) / d_sim
        lidar_hz = len(window_custom) / d_sim
        cloud2_hz = len(window_cloud2) / d_sim

        print(f'=== Verify ({tag}) ===')
        print(
            f'RTF={rtf:.3f} | lidar={lidar_hz:.2f}Hz, cloud2={cloud2_hz:.2f}Hz, '
            f'imu={imu_hz:.1f}Hz, odom={odom_hz:.1f}Hz, joints={joint_hz:.1f}Hz'
        )
        assert rtf >= 0.85, f'RTF too low: {rtf:.3f}'
        assert 9.3 <= lidar_hz <= 10.7, f'Unexpected lidar rate: {lidar_hz:.2f}'
        assert 9.3 <= cloud2_hz <= 10.7, f'Unexpected cloud2 rate: {cloud2_hz:.2f}'
        assert 190.0 <= imu_hz <= 210.0, f'Unexpected imu rate: {imu_hz:.1f}'
        assert 45.0 <= odom_hz <= 55.0, f'Unexpected ground_truth odom rate: {odom_hz:.1f}'
        assert 45.0 <= joint_hz <= 55.0, f'Unexpected joint_states rate: {joint_hz:.1f}'

        # Check zero /tf conflict with LIO (no world->base_link or odom->base_link on /tf)
        forbidden_tf = {('world', 'base_link'), ('odom', 'base_link'), ('world', 'mesh'), ('odom', 'mesh')}
        tf_overlap = node.tf_pairs.intersection(forbidden_tf)
        assert not tf_overlap, f'Forbidden TF published (would conflict with Super-LIO): {tf_overlap}'

        # IMU frame & m/s^2 gravity sanity
        imu_last = window_imu[-1]
        assert imu_last.header.frame_id == 'imu_link', f'Bad IMU frame: {imu_last.header.frame_id}'
        imu_acc = np.array([
            imu_last.linear_acceleration.x,
            imu_last.linear_acceleration.y,
            imu_last.linear_acceleration.z,
        ])
        acc_norm = float(np.linalg.norm(imu_acc))
        assert 9.2 <= acc_norm <= 10.4 and imu_acc[2] > 9.0, f'Unexpected IMU accel (m/s^2): {imu_acc}'

        # Ground-truth odometry validation + static drift
        odom_first = window_odom[0]
        odom_last = window_odom[-1]
        assert odom_last.header.frame_id == 'world', f'Bad odom frame: {odom_last.header.frame_id}'
        assert odom_last.child_frame_id == 'base_link', f'Bad odom child frame: {odom_last.child_frame_id}'
        p_start = np.array([
            odom_first.pose.pose.position.x,
            odom_first.pose.pose.position.y,
            odom_first.pose.pose.position.z,
        ])
        p_end = np.array([
            odom_last.pose.pose.position.x,
            odom_last.pose.pose.position.y,
            odom_last.pose.pose.position.z,
        ])
        static_drift_m = float(np.linalg.norm(p_end - p_start))
        gz_poses_t1 = get_gz_model_poses(['robot'] + dyn_names)
        gz_robot_pos, gz_robot_quat = gz_poses_t1['robot']
        odom_vs_gz_err = float(np.linalg.norm(p_end - gz_robot_pos))
        print(
            f'Ground-truth odom OK: pos=({p_end[0]:.3f},{p_end[1]:.3f},{p_end[2]:.3f}), '
            f'static_drift_3s={static_drift_m*1000:.2f}mm, vs_gz_pose={odom_vs_gz_err*1000:.2f}mm'
        )
        assert static_drift_m < 0.005, f'Static drift too large: {static_drift_m*1000:.2f}mm'
        assert odom_vs_gz_err < 0.010, f'Odom pose disagrees with Gazebo pose: {odom_vs_gz_err*1000:.2f}mm'

        # CustomMsg validation
        recv_sim_t, msg = window_custom[-1]
        assert msg.header.frame_id == 'livox_frame', f'Bad lidar frame: {msg.header.frame_id}'
        assert msg.point_num == 20000 and len(msg.points) == 20000
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        assert msg.timebase == stamp_ns

        offsets = np.array([p.offset_time for p in msg.points], dtype=np.int64)
        uniq_offsets = np.unique(offsets)
        assert np.all(np.diff(offsets) >= 0), 'offset_time not monotonically non-decreasing'
        assert offsets[0] == 0 and len(uniq_offsets) == 5
        assert 70_000_000 <= offsets[-1] <= 90_000_000
        assert (stamp_ns + int(offsets[-1])) * 1e-9 <= recv_sim_t + 0.005, 'Future timestamp detected'

        lines = np.array([p.line for p in msg.points], dtype=np.uint8)
        tags = np.array([p.tag for p in msg.points], dtype=np.uint8)
        refls = np.array([p.reflectivity for p in msg.points], dtype=np.uint8)
        xyz = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float64)
        dists = np.linalg.norm(xyz, axis=1)
        valid_mask = dists > 0.0
        valid_dists = dists[valid_mask]

        assert np.all(tags == 0)
        assert set(np.unique(lines).tolist()) == {0, 1, 2, 3}
        assert np.all(refls[~valid_mask] == 0)
        assert len(valid_dists) > 1000
        assert valid_dists.min() >= 0.1 and valid_dists.max() < 40.0

        if world == 'RMUC2027':
            assert int(np.sum(valid_dists < 0.40)) == 0, 'Self-occlusion hits detected on RMUC2027'

        # Non-repetitive pattern check
        _, prev_msg = window_custom[-2]
        xyz_prev = np.array([[p.x, p.y, p.z] for p in prev_msg.points], dtype=np.float64)
        both_v = (dists > 0.5) & (np.linalg.norm(xyz_prev, axis=1) > 0.5)
        mean_diff = float(np.mean(np.linalg.norm(xyz[both_v] - xyz_prev[both_v], axis=1)))
        assert mean_diff > 0.5, f'Consecutive frames look repetitive: delta={mean_diff:.3f}m'

        # 26-byte LivoxPointXyzrtlt PointCloud2 parity check
        matching_c2 = [c for c in window_cloud2 if c.header.stamp == msg.header.stamp]
        assert len(matching_c2) == 1, 'Missing matching PointCloud2 for CustomMsg'
        c2 = matching_c2[0]
        assert c2.header.frame_id == 'livox_frame' and c2.is_dense is True and c2.point_step == 26
        assert [f.name for f in c2.fields] == ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp']
        assert c2.width == int(valid_mask.sum())
        c2_arr = np.frombuffer(c2.data, dtype=LIVOX_PC2_DTYPE)
        c2_xyz = np.column_stack([c2_arr['x'], c2_arr['y'], c2_arr['z']])
        assert float(np.max(np.abs(c2_xyz - xyz[valid_mask]))) < 1e-5
        assert np.all(c2_arr['intensity'] == refls[valid_mask])
        assert np.all(c2_arr['tag'] == tags[valid_mask])
        assert np.all(c2_arr['line'] == lines[valid_mask])
        expected_ts = (stamp_ns + offsets[valid_mask]).astype(np.float64)
        assert float(np.max(np.abs(c2_arr['timestamp'] - expected_ts))) <= 1024.0

        print(
            f'LiDAR OK: valid={valid_mask.sum()}/20000, '
            f'range=[{valid_dists.min():.2f}, {valid_dists.max():.2f}]m, '
            f'refls={sorted(set(refls[valid_mask].tolist()))}'
        )

        # Dynamic obstacle check: Gazebo motion + GPU lidar reflectivity=120 hits
        if dynamic_obstacles:
            for nm in dyn_names:
                assert nm in gz_poses_t0 and nm in gz_poses_t1, f'Missing dynamic obstacle {nm}'
                disp = float(np.linalg.norm(gz_poses_t1[nm][0] - gz_poses_t0[nm][0]))
                print(f'Dynamic obstacle {nm} displacement over 3s: {disp:.3f}m')
                assert disp > 0.25, f'Dynamic obstacle {nm} did not move enough: {disp:.3f}m'
            retro_hits = int(np.sum(refls[valid_mask] == 120))
            print(f'GPU lidar <laser_retro=120> hits on dynamic obstacles: {retro_hits}')
            assert retro_hits >= 5, f'GPU lidar did not detect dynamic obstacles (retro=120 hits={retro_hits})'

        # Holonomic motion + 5-subframe deskew check
        if check_motion:
            # 1) Pure yaw spin -> 5-subframe deskew alignment against IMU wz
            spin_for_sim(node, 0.8, cmd=(0.0, 0.0, 1.5))
            c_rot_idx = len(node.custom_msgs)
            spin_for_sim(node, 0.6, cmd=(0.0, 0.0, 1.5))
            c_rot_end = len(node.custom_msgs)
            spin_for_sim(node, 0.5, cmd=(0.0, 0.0, 0.0))

            rot_slice = node.custom_msgs[c_rot_idx:c_rot_end]
            assert len(rot_slice) >= 3
            rot_msg = rot_slice[len(rot_slice) // 2][1]
            rot_t0 = rot_msg.timebase * 1e-9
            pts = np.array([[p.x, p.y, p.z, p.offset_time] for p in rot_msg.points], dtype=np.float64)
            u_off = np.unique(pts[:, 3])
            # Pick earliest and latest subframes with >=100 valid hits (>0.5m)
            # (High-elevation subframes in mid360.csv can point at open sky in roofless arenas)
            sf_pts = []
            for off_ns in u_off:
                sp = pts[pts[:, 3] == off_ns, :3]
                sp = sp[np.linalg.norm(sp, axis=1) > 0.5]
                if len(sp) >= 100:
                    sf_pts.append((off_ns, sp))
            assert len(sf_pts) >= 2, f'Too few populated subframes for deskew check: {len(sf_pts)}'
            off_a, p_a = sf_pts[0]
            off_b, p_b = sf_pts[-1]
            dt_s = (off_b - off_a) * 1e-9
            assert dt_s >= 0.035, f'Subframe span too small for deskew check: {dt_s*1000:.1f}ms'
            win_t0 = rot_t0 + off_a * 1e-9
            win_t1 = rot_t0 + off_b * 1e-9
            scan_imu = [
                m.angular_velocity.z for m in node.imu_msgs
                if win_t0 <= (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9) <= win_t1
            ]
            assert len(scan_imu) >= 6
            imu_wz = float(np.mean(scan_imu))

            def median_nn_dist(deg):
                rad = math.radians(deg)
                c, s = math.cos(rad), math.sin(rad)
                rot = np.column_stack([
                    c * p_b[:, 0] - s * p_b[:, 1],
                    s * p_b[:, 0] + c * p_b[:, 1],
                    p_b[:, 2],
                ])
                diff = rot[:, None, :] - p_a[None, :, :]
                d2 = np.einsum('ijk,ijk->ij', diff, diff)
                return float(np.median(np.sqrt(np.min(d2, axis=1))))

            cand_degs = np.linspace(-15.0, 15.0, 61)
            scores = [median_nn_dist(d) for d in cand_degs]
            best_idx = int(np.argmin(scores))
            best_deg = float(cand_degs[best_idx])
            zero_score = median_nn_dist(0.0)
            best_score = scores[best_idx]
            exp_deg = math.degrees(imu_wz * dt_s)
            print(
                f'5-subframe deskew OK: dt={dt_s*1000:.1f}ms, imu_wz={imu_wz:.3f}rad/s, '
                f'best={best_deg:+.1f}deg (exp {exp_deg:+.1f}deg), '
                f'NN={zero_score*1000:.1f}->{best_score*1000:.1f}mm'
            )
            assert abs(best_deg - exp_deg) <= 1.5
            assert best_score < zero_score * 0.75

            # 2) Forward (+X) and strafe (+Y) verification in robot body frame via /sim/ground_truth/odom
            def odom_pose_xy_yaw():
                om = node.odom_msgs[-1].pose.pose
                q = om.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return np.array([om.position.x, om.position.y]), yaw

            p0_xy, yaw0 = odom_pose_xy_yaw()
            spin_for_sim(node, 0.8, cmd=(0.8, 0.0, 0.0))
            p1_xy, yaw1 = odom_pose_xy_yaw()
            spin_for_sim(node, 0.8, cmd=(0.0, 0.8, 0.0))
            p2_xy, _ = odom_pose_xy_yaw()
            spin_for_sim(node, 0.3, cmd=(0.0, 0.0, 0.0))

            dp_x_world = p1_xy - p0_xy
            dx_body = math.cos(yaw0) * dp_x_world[0] + math.sin(yaw0) * dp_x_world[1]
            dp_y_world = p2_xy - p1_xy
            dy_body = -math.sin(yaw1) * dp_y_world[0] + math.cos(yaw1) * dp_y_world[1]
            print(f'Holonomic drive odom check: dx_body(+X cmd)={dx_body:+.3f}m, dy_body(+Y cmd)={dy_body:+.3f}m')
            assert dx_body > 0.35, f'Forward body displacement too small: {dx_body:.3f}m'
            assert dy_body > 0.35, f'Strafe body displacement too small: {dy_body:.3f}m'

        # STL raycast interior accuracy check (RMUC2027)
        if check_geometry:
            R = quat_to_rot(gz_robot_quat)
            lidar_world_pos = gz_robot_pos + R @ np.array([0.0, 0.0, 0.15])
            v0, e1, e2 = load_stl_triangles(stl_path, np.array([10.025, 3.783, -1.398]))
            valid_idx = np.where(valid_mask)[0]
            sample_idx = valid_idx[::max(1, len(valid_idx) // 160)][:160]
            interior_errs = []
            d_ang = math.radians(0.45)
            for idx in sample_idx:
                p_s = xyz[idx]
                r_sim = dists[idx]
                d_s = p_s / r_sim
                r_gt = raycast_stl(lidar_world_pos, R @ d_s, v0, e1, e2)
                if not math.isfinite(r_gt):
                    continue
                az = math.atan2(d_s[1], d_s[0])
                el = math.asin(max(-1.0, min(1.0, d_s[2])))
                nbs = []
                for da, de in [(-d_ang, 0), (d_ang, 0), (0, -d_ang), (0, d_ang)]:
                    v_nb = np.array([
                        math.cos(el + de) * math.cos(az + da),
                        math.cos(el + de) * math.sin(az + da),
                        math.sin(el + de),
                    ])
                    nbs.append(raycast_stl(lidar_world_pos, R @ v_nb, v0, e1, e2))
                if all(math.isfinite(x) for x in nbs) and (max(nbs) - min(nbs) < 0.08):
                    interior_errs.append(r_sim - r_gt)
            interior_errs = np.array(interior_errs)
            print(
                f'STL interior raycast (n={len(interior_errs)}, noise={noise_std}): '
                f'med|e|={np.median(np.abs(interior_errs))*1000:.2f}mm, '
                f'bias={np.mean(interior_errs)*1000:+.2f}mm, std={np.std(interior_errs)*1000:.2f}mm'
            )
            assert len(interior_errs) >= 60
            if noise_std == 0.0:
                assert np.median(np.abs(interior_errs)) < 0.005
                assert np.std(interior_errs) < 0.010
            else:
                assert 0.014 <= np.std(interior_errs) <= 0.028
                assert abs(np.mean(interior_errs)) < 0.010

        # Check launch log for errors
        log_f.flush()
        with open(log_path) as lf:
            log_text = lf.read()
        err_lines = [
            l for l in log_text.splitlines()
            if ('[Err]' in l or 'FATAL' in l or 'ERROR' in l) and 'process has died' not in l
        ]
        assert not err_lines, f'Error lines found in {log_path}:\n' + '\n'.join(err_lines[:10])
        print(f'PASS: {tag}\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=8)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
        log_f.close()
        time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description='Headless verification for pb_rm_simulation (Gazebo Harmonic)')
    parser.add_argument('--world', default='ALL', help='RMUC2027, RMUC, RMUL, RMUL2026H, or ALL')
    parser.add_argument('--dynamic-obstacles', action='store_true', help='Enable dynamic obstacles for single world')
    parser.add_argument('--noise', type=float, default=0.02, help='Lidar range noise stddev for single world')
    parser.add_argument('--check-geometry', action='store_true', help='Run STL raycast accuracy check (RMUC2027)')
    parser.add_argument('--check-motion', action='store_true', help='Run holonomic drive + 5-subframe deskew check')
    args = parser.parse_args()

    if args.world == 'ALL':
        run_case('RMUC2027', noise_std=0.0, check_geometry=True, check_motion=False)
        run_case('RMUC2027', noise_std=0.02, check_geometry=True, check_motion=True)
        run_case('RMUC', noise_std=0.02)
        run_case('RMUL', noise_std=0.02, dynamic_obstacles=True)
        run_case('RMUL2026H', noise_std=0.02, dynamic_obstacles=True)
        print('ALL HARMONIC SIMULATION VERIFICATION SUITES PASSED!')
    else:
        run_case(
            world=args.world,
            noise_std=args.noise,
            dynamic_obstacles=args.dynamic_obstacles,
            check_geometry=args.check_geometry,
            check_motion=args.check_motion,
        )


if __name__ == '__main__':
    main()
