#!/usr/bin/env python3
"""
UAV-X Individual Drone Node v4 (Foolproof PoC Edition)
IIT Bombay Techfest 2026-27 — UAV-X: Resilient BVLOS Swarm Challenge
"""

from __future__ import annotations
import argparse, csv, json, math, random, socket, time
from collections import deque
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Set, Tuple
import xml.etree.ElementTree as ET

try:
    import numpy as np
except ImportError:
    np = None

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from px4_msgs.msg import (
        OffboardControlMode, TrajectorySetpoint, VehicleCommand,
        VehicleLocalPosition, VehicleStatus, BatteryStatus, DistanceSensor
    )
except ImportError:
    rclpy = None
    Node = object

# -----------------------------------------------------------------------------
# Math / Utility
# -----------------------------------------------------------------------------
def clamp01(x: float) -> float: return max(0.0, min(1.0, x))
def safe_battery(value: float) -> float:
    if value is None or value < 0: return 1.0
    return clamp01(value / 100.0)
def finite_or(value: float, default: float) -> float: return value if math.isfinite(value) else default

@dataclass
class Vec3:
    x: float = 0.0; y: float = 0.0; z: float = 0.0
    def __add__(self, o): return Vec3(self.x+o.x, self.y+o.y, self.z+o.z)
    def __sub__(self, o): return Vec3(self.x-o.x, self.y-o.y, self.z-o.z)
    def __mul__(self, s): return Vec3(self.x*s, self.y*s, self.z*s)
    __rmul__ = __mul__
    def __truediv__(self, s):
        if abs(s) < 1e-12: return Vec3()
        return self * (1.0 / s)
    def __neg__(self): return Vec3(-self.x, -self.y, -self.z)
    def norm(self): return math.sqrt(self.x**2 + self.y**2 + self.z**2)
    def hnorm(self): return math.hypot(self.x, self.y)
    def normalized(self): return self / self.norm() if self.norm() > 1e-9 else Vec3()
    def hnormalized(self):
        n = self.hnorm()
        return Vec3(self.x/n, self.y/n, 0.0) if n > 1e-9 else Vec3()
    def clamp(self, max_norm):
        n = self.norm()
        if n > max_norm and n > 1e-9: return self * (max_norm / n)
        return self

def dist(a: Vec3, b: Vec3) -> float: return (a - b).norm()
def mean_vec(vectors: Sequence[Vec3]) -> Vec3:
    if not vectors: return Vec3()
    total = Vec3()
    for v in vectors: total += v
    return total / len(vectors)

# -----------------------------------------------------------------------------
# States / Data Model
# -----------------------------------------------------------------------------
class DroneMode(Enum):
    INIT="INIT"; NORMAL="NORMAL"; LOW_BATTERY_WARNING="LOW_BATTERY_WARNING"
    RETURN_REQUIRED="RETURN_REQUIRED"; RTH="RTH"; FAULT_RECOVERY="FAULT_RECOVERY"; COMPLETE="COMPLETE"

class PoiStatus(Enum):
    PENDING="PENDING"; ASSIGNED="ASSIGNED"; IN_PROGRESS="IN_PROGRESS"; COMPLETED="COMPLETED"
    FAILED="FAILED"; REASSIGN_REQUIRED="REASSIGN_REQUIRED"; UNREACHABLE="UNREACHABLE"; DEFERRED="DEFERRED"

@dataclass
class Poi:
    poi_id: str; x: float; y: float; z: float; priority: int = 1
    status: PoiStatus = PoiStatus.PENDING
    assigned_uav: Optional[int] = None
    @property
    def pos(self) -> Vec3: return Vec3(self.x, self.y, self.z)

@dataclass(frozen=True)
class SimObstacle:
    shape: str
    x: float
    y: float
    radius: float = 0.0
    width: float = 0.0
    depth: float = 0.0
    yaw: float = 0.0

@dataclass
class Telemetry:
    position: Vec3; velocity: Vec3; battery_pct: float = -1.0
    armed: bool = False; mode: str = "UNKNOWN"; healthy: bool = True
    tof_distance_m: float = float("inf")

@dataclass
class Peer:
    uav_id: int; position: Vec3; velocity: Vec3; battery_pct: float; mode: str
    healthy: bool; poi_id: Optional[str]; poi_status: str; poi_priority: int
    obstacle_distance: float; proposed_direction: Vec3; cluster_id: str
    last_rx: float; sent_ts: float; tx_seq: int = 0
    last_distance: float = 0.0; link_quality_db: float = -999.0; connected: bool = False
    pdr_ewma: float = 1.0; link_lifetime_s: float = float("inf")
    gcs_direct: bool = False; gcs_hop_count: Optional[int] = None
    gcs_route_quality_db: float = -999.0; gcs_route_pdr: float = 0.0
    signal_rate_db_s: float = 0.0; degradation_rate: float = 0.0
    cluster_role: str = "member"   # "member" | "ch" | "gch"
    group_role: str = "member"     # "member" | "ch" | "gch"
    neighbor_ids: Set[int] = field(default_factory=set); last_beacon: float = 0.0

@dataclass
class RouteEntry:
    next_hop: Optional[int]; hop_count: Optional[int]; route_quality_db: float
    pdr: float; predicted_lifetime_s: float; updated_at: float

@dataclass
class DataPacket:
    packet_id: str; origin_uav: int; final_dst: str; payload: Dict; ttl: int
    path: List[int]; created_ts: float; last_try_ts: float = 0.0; attempts: int = 0
    def to_dict(self) -> Dict:
        return {"packet_id": self.packet_id, "origin_uav": self.origin_uav, "final_dst": self.final_dst,
                "payload": self.payload, "ttl": self.ttl, "path": list(self.path),
                "created_ts": self.created_ts, "last_try_ts": self.last_try_ts, "attempts": self.attempts}

@dataclass
class Config:
    uav_id: int = 1; swarm_size: int = 5
    formation_radius_m: float = 20.0; fluid_outer_radius_factor: float = 1.25
    fluid_inner_radius_factor: float = 0.45; fluid_cohesion_gain: float = 0.55
    fluid_separation_gain: float = 1.15; fluid_alignment_gain: float = 0.16
    mission_attraction_gain: float = 1.0; connectivity_gain: float = 1.65
    connectivity_target_margin_m: float = 2.0; network_deform_limit_m: float = 10.0
    adaptive_spacing_gain: float = 0.45
    
    tx_power_dbm: float = 20.0; frequency_hz: float = 2.4e9; path_loss_exponent: float = 2.7
    rx_sensitivity_dbm: float = -90.0; link_margin_db: float = 10.0; buffer_distance_m: float = 5.0
    
    route_max_hops: int = 8; route_entry_timeout_s: float = 4.0; packet_ttl: int = 12
    max_forward_queue: int = 128; duplicate_cache_size: int = 2048; packet_retry_period_s: float = 0.5
    link_lifetime_floor_s: float = 0.5; routing_w_link: float = 0.30; routing_w_pdr: float = 0.20
    routing_w_progress: float = 0.20; routing_w_lifetime: float = 0.15; routing_w_battery: float = 0.10
    routing_w_hops: float = 0.05
    
    control_hz: float = 5.0; cruise_speed_mps: float = 4.0; max_target_step_m: float = 2.0
    min_separation_m: float = 6.0; prediction_horizon_s: float = 2.5; reaction_threshold_s: float = 2.5
    avoidance_gain: float = 1.6; max_avoidance_offset_m: float = 8.0
    collision_anticipation_factor: float = 1.75; collision_avoidance_multiplier: float = 1.5
    
    cluster_min: int = 2; cluster_max: int = 4; hysteresis_margin_db: float = 3.0; hysteresis_hold_s: float = 2.0
    low_battery_pct: float = 30.0; rth_battery_pct: float = 20.0; peer_timeout_s: float = 2.5
    
    bid_period_s: float = 2.0; bid_consensus_hold_s: float = 0.6; task_preempt_priority_delta: int = 2
    task_priority_weight: float = 0.45; task_energy_weight: float = 0.30; task_connectivity_weight: float = 0.35
    task_progress_weight: float = 0.15
    
    relay_link_weight: float = 0.25; relay_battery_weight: float = 0.15; relay_topology_weight: float = 0.25
    relay_task_weight: float = 0.15; relay_route_weight: float = 0.20
    
    recovery_hold_s: float = 1.0; recovery_timeout_s: float = 10.0; recovery_lookback_s: float = 8.0
    pdr_alpha: float = 0.25
    
    gcs_position: Vec3 = field(default_factory=Vec3)
    peer_host: str = "127.0.0.1"; peer_base_port: int = 16000; gcs_port: int = 15999; log_dir: str = "uav_x_logs"
    poi_complete_radius_m: float = 6.0; cruise_altitude_m: float = 5.0
    cluster_size: int = 3    # members per cluster (2-3)
    group_size: int = 3      # clusters per group (2-3)

    @property
    def pl_d0_db(self) -> float:
        c = 299792458.0
        return 20.0 * math.log10(4.0 * math.pi * self.frequency_hz / c)
    @property
    def r_max_m(self) -> float:
        return 10 ** ((self.tx_power_dbm - self.rx_sensitivity_dbm - self.link_margin_db - self.pl_d0_db) / (10.0 * self.path_loss_exponent))
    @property
    def r_safe_m(self) -> float: return max(0.0, self.r_max_m - self.buffer_distance_m)

# -----------------------------------------------------------------------------
# Link Model
# -----------------------------------------------------------------------------
class LinkModel:
    def __init__(self, cfg: Config): self.cfg = cfg
    def path_loss(self, d_m): d_m = max(1.0, float(d_m)); return self.cfg.pl_d0_db + 10.0 * self.cfg.path_loss_exponent * math.log10(d_m)
    def rx_power(self, d_m): return self.cfg.tx_power_dbm - self.path_loss(d_m)
    def quality(self, d_m): return self.rx_power(d_m) - self.cfg.rx_sensitivity_dbm
    def connected(self, d_m): return self.quality(d_m) >= self.cfg.link_margin_db
    def margin_db(self, d_m): return self.quality(d_m) - self.cfg.link_margin_db
    def pdr_estimate(self, d_m):
        margin = self.margin_db(d_m)
        if margin >= 15: return 0.995
        if margin <= -5: return 0.05
        return clamp01(0.05 + 0.945 * (margin + 5.0) / 20.0)
    def is_neighbor(self, a: Vec3, b: Vec3): return self.connected(dist(a, b))

# -----------------------------------------------------------------------------
# Geofence
# -----------------------------------------------------------------------------
def point_in_polygon(p: Vec3, polygon: Sequence[Vec3]) -> bool:
    if len(polygon) < 3: return True
    inside = False; j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i].x, polygon[i].y; xj, yj = polygon[j].x, polygon[j].y
        if (yi > p.y) != (yj > p.y):
            xcross = (xj - xi) * (p.y - yi) / (yj - yi + 1e-12) + xi
            if p.x < xcross: inside = not inside
        j = i
    return inside

def force_inside(p: Vec3, polygon: Sequence[Vec3]) -> Vec3:
    if len(polygon) < 3 or point_in_polygon(p, polygon): return p
    c = Vec3(sum(pt.x for pt in polygon)/len(polygon), sum(pt.y for pt in polygon)/len(polygon), 0.0)
    lo, hi = 0.0, 1.0
    for _ in range(40):
        mid = (lo + hi) / 2.0; q = p * (1 - mid) + c * mid
        if point_in_polygon(q, polygon): hi = mid
        else: lo = mid
    return p * (1 - hi) + c * hi

# -----------------------------------------------------------------------------
# UDP Swarm Bus
# -----------------------------------------------------------------------------
class SwarmBus:
    def __init__(self, cfg: Config):
        self.cfg = cfg; self.port = cfg.peer_base_port + cfg.uav_id - 1
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((cfg.peer_host, self.port)); self.sock.setblocking(False)
    def _send_raw(self, host, port, payload):
        try: self.sock.sendto(json.dumps(payload, separators=(",", ":")).encode("utf-8"), (host, port)); return True
        except OSError: return False
    def send_all(self, payload, src_uav):
        count = 0
        for uid in range(1, self.cfg.swarm_size + 1):
            if uid != src_uav and self.send_peer(uid, payload): count += 1
        return count
    def send_peer(self, uid, payload):
        if uid == self.cfg.uav_id: return False
        return self._send_raw(self.cfg.peer_host, self.cfg.peer_base_port + uid - 1, payload)
    def send_gcs(self, payload): return self._send_raw(self.cfg.peer_host, self.cfg.gcs_port, payload)
    def recv_all(self):
        res = []
        while True:
            try: raw, _ = self.sock.recvfrom(65535)
            except BlockingIOError: break
            except OSError: break
            try: res.append(json.loads(raw.decode("utf-8")))
            except: continue
        return res
    def close(self): self.sock.close()

# -----------------------------------------------------------------------------
# PX4 ROS2 Interface
# -----------------------------------------------------------------------------
TOPIC_STATUS = "fmu/out/vehicle_status_v4"
TOPIC_LOCAL_POS = "fmu/out/vehicle_local_position_v1"
TOPIC_BATTERY = "fmu/out/battery_status_v1"
TOPIC_DISTANCE = "fmu/out/distance_sensor_v1"
TOPIC_OFFBOARD_MODE = "fmu/in/offboard_control_mode"
TOPIC_TRAJECTORY_SETPOINT = "fmu/in/trajectory_setpoint"
TOPIC_VEHICLE_COMMAND = "fmu/in/vehicle_command"
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0

class PX4Offboard(Node):
    def __init__(self, namespace: str, uav_id: int, origin_offset: Optional[Vec3] = None):
        if rclpy is None: raise RuntimeError("rclpy/px4_msgs not importable. Source ROS2 workspace.")
        if not rclpy.ok(): rclpy.init()
        super().__init__(f"uav_x_node_{uav_id}")
        self.uav_id = uav_id; ns = namespace.strip("/"); self.prefix = f"/{ns}/" if ns else "/"
        self.origin_offset = origin_offset if origin_offset is not None else Vec3()
        
        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_pub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        self._status = None; self._local_pos = None; self._battery = None; self._tof_distance = float("inf")
        self.system_id = 1
        self.create_subscription(VehicleStatus, self.prefix + TOPIC_STATUS, self._on_status, qos_sub)
        self.create_subscription(VehicleLocalPosition, self.prefix + TOPIC_LOCAL_POS, self._on_local_pos, qos_sub)
        self.create_subscription(BatteryStatus, self.prefix + TOPIC_BATTERY, self._on_battery, qos_sub)
        self.create_subscription(DistanceSensor, self.prefix + TOPIC_DISTANCE, self._on_distance, qos_sub)
        
        self.offboard_pub = self.create_publisher(OffboardControlMode, self.prefix + TOPIC_OFFBOARD_MODE, qos_pub)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, self.prefix + TOPIC_TRAJECTORY_SETPOINT, qos_pub)
        self.command_pub = self.create_publisher(VehicleCommand, self.prefix + TOPIC_VEHICLE_COMMAND, qos_pub)

    def _on_status(self, msg):
        self._status = msg
        if hasattr(msg, 'system_id') and msg.system_id > 0: self.system_id = msg.system_id
    def _on_local_pos(self, msg): self._local_pos = msg
    def _on_battery(self, msg): self._battery = msg
    def _on_distance(self, msg):
        if hasattr(msg, 'current_distance'):
            self._tof_distance = float(msg.current_distance)
    def _spin(self, n=1):
        for _ in range(n): rclpy.spin_once(self, timeout_sec=0.0)
    def _now_us(self): return int(self.get_clock().now().nanoseconds / 1000)
    def _to_world(self, position: Vec3) -> Vec3:
        return position + self.origin_offset
    def _to_local(self, position: Vec3) -> Vec3:
        return position - self.origin_offset
    def _send_command(self, command, **params):
        msg = VehicleCommand(); msg.command = command
        msg.param1 = params.get("param1", 0.0); msg.param2 = params.get("param2", 0.0); msg.param3 = params.get("param3", 0.0)
        msg.target_system = self.system_id; msg.target_component = 1
        msg.source_system = 200 + self.uav_id; msg.source_component = 1; msg.from_external = True
        msg.timestamp = self._now_us(); self.command_pub.publish(msg)
    def _publish_offboard_heartbeat(self):
        msg = OffboardControlMode(); msg.position = True; msg.velocity = False; msg.acceleration = False
        msg.attitude = False; msg.body_rate = False; msg.timestamp = self._now_us(); self.offboard_pub.publish(msg)
        
    def heartbeat(self):
        print("[PX4] waiting for vehicle_status...")
        for _ in range(300):
            self._spin(1)
            if self._status is not None:
                print(f"[PX4] connected, nav_state={self._status.nav_state}, system_id={self.system_id}")
                return
            time.sleep(0.1)
        raise TimeoutError("No vehicle_status received. Check agent and namespace.")
    def request_streams(self): pass
    def pump(self, telemetry: Telemetry):
        self._spin(10)
        if self._local_pos is not None:
            lp = self._local_pos
            telemetry.position = self._to_world(Vec3(lp.x, lp.y, lp.z))
            telemetry.velocity = Vec3(lp.vx, lp.vy, lp.vz)
        if self._status is not None:
            telemetry.armed = self._status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            telemetry.mode = str(self._status.nav_state); telemetry.healthy = True
        if self._battery is not None:
            rem = getattr(self._battery, "remaining_percent", getattr(self._battery, "remaining", -1.0))
            if rem is not None and rem >= 0: telemetry.battery_pct = float(rem) * 100.0
        telemetry.tof_distance_m = self._tof_distance
    def guided(self):
        hold = self._local_pos
        hold_local = Vec3(hold.x, hold.y, hold.z) if hold is not None else Vec3()
        hold_vec = self._to_world(hold_local)
        for _ in range(10): self.position_target(hold_vec); self._spin(1); time.sleep(0.1)
        self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        for _ in range(5): self.position_target(hold_vec); self._spin(1); time.sleep(0.1)
    def arm(self):
        print("[PX4] Arming...")
        # Hold the ground position while arming — stream both offboard heartbeat
        # AND a trajectory setpoint every cycle, otherwise PX4 exits offboard mode
        # between retries and the arm command is rejected.
        hold = self._local_pos
        hold_local = Vec3(hold.x, hold.y, hold.z) if hold is not None else Vec3()
        hold_world = self._to_world(hold_local)
        for _ in range(150):  # up to 15 seconds
            self._publish_offboard_heartbeat()
            self.position_target(hold_world)          # keep offboard alive
            self.ensure_offboard()
            self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self._spin(5)
            if self._status is not None and self._status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                print("[PX4] Armed successfully!")
                return
            time.sleep(0.1)
        print("[PX4] WARNING: Failed to arm! Check if pre-flight checks passed.")
    def takeoff(self, alt_m):
        target = Vec3(self.origin_offset.x, self.origin_offset.y, -abs(alt_m))
        print(f"[PX4] Taking off to {alt_m}m...")
        for _ in range(60):  # up to 6 seconds to reach climb
            self.position_target(target)
            self.ensure_offboard()
            self._spin(5)
            if self._local_pos is not None and self._local_pos.z <= -abs(alt_m) * 0.7:
                print(f"[PX4] Takeoff complete, current alt: {-self._local_pos.z:.1f}m")
                return
            time.sleep(0.1)
    def position_target(self, position: Vec3):
        self._publish_offboard_heartbeat()
        local_position = self._to_local(position)
        msg = TrajectorySetpoint(); msg.position = [local_position.x, local_position.y, local_position.z]
        nan = float('nan')
        msg.velocity = [nan, nan, nan]; msg.acceleration = [nan, nan, nan]
        msg.yaw = nan; msg.yawspeed = nan; msg.timestamp = self._now_us()
        self.setpoint_pub.publish(msg)
    def rtl(self): self._send_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
    def ensure_offboard(self):
        if self._status is None: return
        if self._status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD)
    def rtl_and_wait_for_disarm(self, timeout_s=90.0):
        self.rtl()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._spin(10)
            if self._status is not None and self._status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                print("[PX4] RTL complete; vehicle disarmed.")
                return
            time.sleep(0.1)
        raise TimeoutError(f"PX4 UAV {self.uav_id} did not land and disarm within {timeout_s:.0f} seconds.")

# -----------------------------------------------------------------------------
# Task Allocation (CBAA)
# -----------------------------------------------------------------------------
@dataclass
class TaskBid: uav_id: int; bid: float; epoch: int; sent_at: float
@dataclass
class TaskWinner: uav_id: Optional[int]; bid: float; epoch: int; stable_since: float

class TaskManager:
    def __init__(self, cfg: Config, pois: Iterable[Poi]):
        self.cfg = cfg; self.pois = {p.poi_id: p for p in pois}
        self.assigned: Optional[str] = None; self.bid_table: Dict[str, Dict[int, TaskBid]] = {}
        self.winner_table: Dict[str, TaskWinner] = {}; self.local_epoch = 0
        self.last_bid_time = 0.0; self.last_consensus_change = 0.0; self.created_at = time.monotonic()
        self.gcs_override_uid = None; self.vote_start_time = 0.0

    def current(self) -> Optional[Poi]: return self.pois.get(self.assigned) if self.assigned else None
    def is_critical_assignment(self, mode, bridge): return mode == DroneMode.FAULT_RECOVERY or bridge
    def score(self, poi, pos, batt, conn_count, prog=0.0):
        battery = max(5.0, batt if batt >= 0 else 100.0); d = dist(pos, poi.pos)
        norm_pri = max(0.0, poi.priority - 1.0) / 5.0; e_cost = d / battery
        iso_pen = 1.0 if conn_count == 0 else 0.0; prog_cred = clamp01(prog)
        return (d + self.cfg.task_energy_weight * 100.0 * e_cost + self.cfg.task_connectivity_weight * 10.0 * iso_pen
                - self.cfg.task_priority_weight * 20.0 * norm_pri - self.cfg.task_progress_weight * 10.0 * prog_cred)
    def register_bid(self, poi_id, uid, bid, epoch, now):
        table = self.bid_table.setdefault(poi_id, {}); ex = table.get(uid)
        if ex is None or epoch >= ex.epoch: table[uid] = TaskBid(uid, bid, epoch, now)
    def recompute_winner(self, poi_id, now):
        bids = self.bid_table.get(poi_id, {})
        if not bids: return None
        fresh = [b for b in bids.values() if now - b.sent_at <= self.cfg.peer_timeout_s + self.cfg.bid_period_s]
        if not fresh: return None
        best = min(fresh, key=lambda b: (b.bid, b.uav_id)); old = self.winner_table.get(poi_id)
        if old is None or old.uav_id != best.uav_id:
            w = TaskWinner(best.uav_id, best.bid, best.epoch, now); self.winner_table[poi_id] = w
            self.last_consensus_change = now; return w
        old.bid = best.bid; old.epoch = best.epoch; return old
    def should_preempt(self, curr, chal, mode, bridge):
        if curr is None or chal is None: return True
        if self.is_critical_assignment(mode, bridge): return False
        return chal.priority >= curr.priority + self.cfg.task_preempt_priority_delta
    def add_priority_poi(self, poi):
        ex = self.pois.get(poi.poi_id)
        if ex is None or poi.priority > ex.priority: self.pois[poi.poi_id] = poi
    def release_current(self):
        p = self.current()
        if p is not None and p.status != PoiStatus.COMPLETED: p.status = PoiStatus.REASSIGN_REQUIRED; p.assigned_uav = None
        old = self.assigned; self.assigned = None
        if old: self.local_epoch += 1; self.bid_table.pop(old, None); self.winner_table.pop(old, None)
        return old
    def complete_current(self, uid):
        p = self.current()
        if p is None: return None
        p.status = PoiStatus.COMPLETED; p.assigned_uav = uid; old = self.assigned; self.assigned = None; return old
    def local_try_assign(self, uid, pos, batt, conn_count, now, mode, bridge):
        if self.assigned: return self.current()
        cands = [p for p in self.pois.values() if p.status in (PoiStatus.PENDING, PoiStatus.REASSIGN_REQUIRED, PoiStatus.DEFERRED)]
        if not cands: 
            self.vote_start_time = 0.0
            return None
            
        if self.vote_start_time == 0.0:
            self.vote_start_time = now
            
        if now - self.last_bid_time >= self.cfg.bid_period_s: self.local_epoch += 1; self.last_bid_time = now
        
        # GCS Override logic (if voting takes 450ms or more)
        if now - self.vote_start_time >= 0.45:
            import random
            print(f"[TASK] UAV {uid} GCS OVERRIDE triggered (voting took > 450ms). Assigning randomly by severity.")
            total_pri = sum(p.priority for p in cands)
            weights = [p.priority / total_pri for p in cands] if total_pri > 0 else None
            # Use deterministic choice based on time and uav_id so it acts like a global GCS decision
            r = random.Random(int(now) + uid)
            chosen = r.choices(cands, weights=weights, k=1)[0]
            self.assigned = chosen.poi_id
            chosen.assigned_uav = uid
            chosen.status = PoiStatus.ASSIGNED
            self.vote_start_time = 0.0
            return chosen

        eligible = []
        for poi in cands:
            bid = self.score(poi, pos, batt, conn_count)
            self.register_bid(poi.poi_id, uid, bid, self.local_epoch, now)
            winner = self.recompute_winner(poi.poi_id, now)
            if winner is None or winner.uav_id != uid: continue
            known = len(self.bid_table.get(poi.poi_id, {}))
            grace_period = max(5.0, self.cfg.bid_period_s * 2.5)
            grace = (now - self.created_at) >= grace_period and (now - winner.stable_since) >= grace_period
            singleton = conn_count == 0 and known == 1 and grace
             
            if self.gcs_override_uid == uid:
                eligible.append((bid, poi))
            elif singleton or (conn_count > 0 and (now - winner.stable_since) >= self.cfg.bid_consensus_hold_s):
                eligible.append((bid, poi))
                
        if not eligible: return None
        _, chosen = min(eligible, key=lambda item: (item[0], -item[1].priority, item[1].poi_id))
        self.assigned = chosen.poi_id; chosen.assigned_uav = uid; chosen.status = PoiStatus.ASSIGNED
        self.vote_start_time = 0.0
        print(f"[TASK] UAV {uid} consensus-claimed {chosen.poi_id}")
        return chosen
    def progress_fraction(self, pos):
        p = self.current()
        if p is None: return 0.0
        return clamp01(1.0 - dist(pos, p.pos) / max(1.0, self.cfg.formation_radius_m * 5.0))
    def mark_progress(self, pos, uid):
        p = self.current()
        if p is None: return None
        if p.status == PoiStatus.ASSIGNED: p.status = PoiStatus.IN_PROGRESS
        if dist(pos, p.pos) <= self.cfg.poi_complete_radius_m:
            old = self.complete_current(uid); print(f"[TASK] UAV {uid} completed {p.poi_id}"); return old
        return None

# -----------------------------------------------------------------------------
# Obstacles (Stereo / ToF Sensor Fusion)
# -----------------------------------------------------------------------------
class NullObstacleProvider:
    def __init__(self):
        self.last_valid = True
        self.last_confidence = 1.0
    def update(self, pos, gdir, tof=float("inf")):
        if not math.isfinite(tof) or tof <= 0.0:
            self.last_valid = False
            self.last_confidence = 0.0
            return float("inf"), gdir.normalized()
        self.last_valid = 0.2 <= tof <= 12.0
        self.last_confidence = clamp01((12.0 - tof) / 10.0) if tof <= 12.0 else 0.8
        if tof > 12.0:
            return tof, gdir.normalized()
        direction = gdir.normalized()
        lateral = Vec3(-gdir.y, gdir.x, 0.0).hnormalized()
        if lateral.norm() < 1e-9: lateral = Vec3(1.0, 0.0, 0.0)
        closeness = clamp01((12.0 - tof) / 12.0)
        return tof, (direction * (1.0 - closeness) + lateral * closeness).normalized()

class SimStereoToFSensor:
    """Ray-casts against the same cylinder, sphere, and box footprints used by Gazebo."""
    def __init__(self, obstacles: Sequence[SimObstacle]):
        self.obstacles = list(obstacles)
        self.last_tof = float("inf")
        self.last_valid = True
        self.last_confidence = 1.0

    def validate_optical_depth(self, tof_reading: float, dt: float = 0.2) -> Tuple[bool, float, float]:
        """Applies edge detection and noise filtering to reject invalid or motion-blurred depth readings."""
        if not math.isfinite(tof_reading) or tof_reading < 0.2 or tof_reading > 15.0:
            return False, float("inf"), 0.0
        if math.isfinite(self.last_tof) and self.last_tof > 0.0:
            jump_rate = abs(tof_reading - self.last_tof) / max(1e-3, dt)
            if jump_rate > 35.0:
                clamped = self.last_tof + math.copysign(35.0 * dt, tof_reading - self.last_tof)
                self.last_tof = clamped
                return True, clamped, 0.40
        self.last_tof = float(tof_reading)
        confidence = clamp01((12.0 - tof_reading) / 10.0) if tof_reading <= 12.0 else 0.85
        return True, float(tof_reading), confidence

    @staticmethod
    def _to_obstacle_frame(obstacle: SimObstacle, x: float, y: float) -> Tuple[float, float]:
        dx = x - obstacle.x
        dy = y - obstacle.y
        c = math.cos(obstacle.yaw)
        s = math.sin(obstacle.yaw)
        return c * dx + s * dy, -s * dx + c * dy

    @staticmethod
    def _from_obstacle_frame(obstacle: SimObstacle, x: float, y: float) -> Vec3:
        c = math.cos(obstacle.yaw)
        s = math.sin(obstacle.yaw)
        return Vec3(c * x - s * y, s * x + c * y, 0.0)

    @staticmethod
    def _box_ray_distance(ox: float, oy: float, dx: float, dy: float,
                          half_width: float, half_depth: float) -> float:
        t_enter = float("-inf")
        t_exit = float("inf")
        for origin, direction, extent in (
            (ox, dx, half_width),
            (oy, dy, half_depth),
        ):
            if abs(direction) < 1e-9:
                if abs(origin) > extent:
                    return float("inf")
                continue
            t1 = (-extent - origin) / direction
            t2 = (extent - origin) / direction
            t_enter = max(t_enter, min(t1, t2))
            t_exit = min(t_exit, max(t1, t2))
        if t_exit < max(t_enter, 0.0):
            return float("inf")
        return max(t_enter, 0.0)

    def cast_ray(self, pos: Vec3, direction: Vec3) -> float:
        """Returns the distance to the nearest obstacle along the ray."""
        min_dist = float("inf")
        for obstacle in self.obstacles:
            ox, oy = self._to_obstacle_frame(obstacle, pos.x, pos.y)
            local_dx, local_dy = self._to_obstacle_frame(
                obstacle, obstacle.x + direction.x, obstacle.y + direction.y
            )
            if obstacle.shape == "box":
                hit_dist = self._box_ray_distance(
                    ox, oy, local_dx, local_dy,
                    obstacle.width / 2.0, obstacle.depth / 2.0,
                )
            else:
                radius = obstacle.radius
                projection = -(ox * local_dx + oy * local_dy)
                discriminant = projection * projection - (ox * ox + oy * oy - radius * radius)
                if discriminant < 0.0:
                    continue
                if ox * ox + oy * oy <= radius * radius:
                    hit_dist = 0.0
                else:
                    near_hit = projection - math.sqrt(discriminant)
                    far_hit = projection + math.sqrt(discriminant)
                    hit_dist = near_hit if near_hit >= 0.0 else far_hit
                if hit_dist < 0.0:
                    continue
            min_dist = min(min_dist, hit_dist)
        return min_dist

    def _surface_distance_and_away(self, obstacle: SimObstacle, pos: Vec3,
                                   gdir: Vec3) -> Tuple[float, Vec3]:
        x, y = self._to_obstacle_frame(obstacle, pos.x, pos.y)
        if obstacle.shape != "box":
            distance = math.hypot(x, y)
            away = Vec3(x, y, 0.0).hnormalized()
            if away.norm() < 1e-9:
                away = Vec3(-gdir.y, gdir.x, 0.0).hnormalized()
            return distance - obstacle.radius, self._from_obstacle_frame(obstacle, away.x, away.y)

        half_width = obstacle.width / 2.0
        half_depth = obstacle.depth / 2.0
        outside_x = max(abs(x) - half_width, 0.0)
        outside_y = max(abs(y) - half_depth, 0.0)
        if outside_x > 0.0 or outside_y > 0.0:
            distance = math.hypot(outside_x, outside_y)
            away_x = math.copysign(outside_x, x) if outside_x else 0.0
            away_y = math.copysign(outside_y, y) if outside_y else 0.0
        elif half_width - abs(x) <= half_depth - abs(y):
            distance = -(half_width - abs(x))
            away_x = math.copysign(1.0, x if x else 1.0)
            away_y = 0.0
        else:
            distance = -(half_depth - abs(y))
            away_x = 0.0
            away_y = math.copysign(1.0, y if y else 1.0)
        away = Vec3(away_x, away_y, 0.0).hnormalized()
        return distance, self._from_obstacle_frame(obstacle, away.x, away.y)

    def update(self, pos: Vec3, gdir: Vec3, tof_reading: float = float("inf")) -> Tuple[float, Vec3]:
        sim_dist = self.cast_ray(pos, gdir.hnormalized())
        tof_valid, tof_dist, conf = self.validate_optical_depth(tof_reading)
        self.last_valid = tof_valid
        self.last_confidence = conf
        sensor_dist = min(sim_dist, tof_dist)

        best_analytical_d = float("inf")
        lateral = Vec3(-gdir.y, gdir.x, 0.0).hnormalized()
        if lateral.norm() < 1e-9: lateral = Vec3(1.0, 0.0, 0.0)
        best_away = lateral if tof_dist <= sim_dist else gdir.normalized()
        for obstacle in self.obstacles:
            surf_d, away = self._surface_distance_and_away(obstacle, pos, gdir)
            if surf_d < best_analytical_d:
                best_analytical_d = surf_d
                best_away = away

        avoidance_distance = min(best_analytical_d, sensor_dist)
        if avoidance_distance > 12.0:
            return avoidance_distance, gdir.normalized()

        closeness = clamp01((12.0 - max(avoidance_distance, 0.0)) / 12.0)
        direction = (gdir.normalized() * (1.0 - closeness) + best_away * closeness).normalized()
        return max(0.0, avoidance_distance), direction

# -----------------------------------------------------------------------------
# Individual Drone
# -----------------------------------------------------------------------------
class IndividualDrone:
    def __init__(self, cfg, ap, pois, geofence, obstacle, dry_run):
        self.cfg = cfg; self.ap = ap; self.t = Telemetry(Vec3(), Vec3())
        self.task = TaskManager(cfg, pois); self.fence = list(geofence); self.obstacle = obstacle
        self.dry_run = dry_run; self.mode = DroneMode.INIT; self.link = LinkModel(cfg)
        self.bus = SwarmBus(cfg); self.peers = {}; self.cluster_id = f"SOLO-{cfg.uav_id}"
        
        self.gcs_direct = False; self.gcs_hop_count = None; self.gcs_route_quality_db = -999.0
        self.gcs_route_pdr = 0.0; self.next_hop_id = None; self.route_entry = None; self.neighbor_hops = {}
        self.forward_queue = deque(maxlen=cfg.max_forward_queue); self.seen_packets = deque(maxlen=cfg.duplicate_cache_size); self.seen_packet_set = set()
        
        self.tx_seq = 0; self.beacon_seq = 0; self.data_seq = 0
        self.target = Vec3(); self.recovery_target = None; self.prev_peer_dist = {}; self.prev_peer_time = {}
        self.link_at_risk = False; self.collision_violation = False; self.geofence_violation = False
        self.rth_triggered = False; self.last_relay = None; self.relay_changes = 0
        self.bridge_active = False; self.bridge_target = None; self.recovery_event = ""
        self.recovery_started_at = None; self.recovery_count = 0; self.recovery_time_last_s = 0.0
        self.disconnect_started_at = None; self.connectivity_downtime_s = 0.0
        self.last_lambda2 = 0.0; self.relay_candidate_id = None; self.relay_candidate_since = 0.0
        self.last_relay_switch_ts = 0.0; self.speed_scale = 1.0
        self.cluster_role = "member"; self.group_role = "member"
        self.locked_poi: Optional[str] = None
        cluster_num = (cfg.uav_id - 1) // max(1, cfg.cluster_size) + 1
        self.cluster_id = f"CLUSTER-{cluster_num}"
        self.group_id = "GROUP-1"
        self.flight_alt_m = cfg.cruise_altitude_m + ((cfg.uav_id - 1) % 3) * 1.5
        self.last_completed_poi_pos: Optional[Vec3] = None  # hold-position after POI done
        
        self.tx_packets = 0; self.rx_packets = 0; self.forwarded_packets = 0; self.delivered_to_gcs = 0
        self.dropped_packets = 0; self.latencies_ms = deque(maxlen=500); self.pdr_tx = 0; self.pdr_rx = 0
        self.formation_error = 0.0; self.network_deformation = 0.0; self.nearest_neighbor = float("inf")
        self.last_state = 0.0; self.last_beacon = 0.0; self.last_bid = 0.0; self.last_burst = 0.0
        self.random = random.Random(1000 + cfg.uav_id); self.total_cycles = 0; self.events = {}
        
        Path(cfg.log_dir).mkdir(parents=True, exist_ok=True)
        self.log_path = Path(cfg.log_dir) / f"uav_{cfg.uav_id}_log.csv"
        self.log = self.log_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.log, fieldnames=[
            "timestamp", "uav_id", "mode", "x_n", "y_e", "z_d", "vx", "vy", "vz", "battery_pct",
            "armed", "flight_mode", "poi_id", "poi_status", "cluster_id", "neighbor_count",
            "connected_neighbors", "nearest_neighbor_m", "next_hop", "gcs_direct", "gcs_reachable",
            "gcs_hop_count", "route_quality_db", "route_pdr", "strongest_relay", "relay_score",
            "obstacle_distance_m", "formation_error_m", "network_deformation_m", "minimum_separation_m",
            "geofence_violation", "collision_violation", "link_at_risk", "rth_triggered",
            "recovery_event", "recovery_count", "recovery_time_last_s", "pdr_tx", "pdr_rx",
            "forwarded_packets", "delivered_to_gcs", "dropped_packets", "latency_ms_avg",
            "connectivity_downtime_s", "relay_reallocations", "bridge_active"
        ])
        self.writer.writeheader()

    def _new_envelope(self, payload, kind, dst_uav=None):
        self.tx_seq += 1; env = dict(payload)
        env.setdefault("src_uav", self.cfg.uav_id); env.setdefault("sent_ts", time.time())
        env.setdefault("seq", self.tx_seq); env.setdefault("type", kind)
        if dst_uav is not None: env["dst_uav"] = dst_uav
        return env
    def broadcast(self, kind, payload):
        if not self.bus: return 0
        msg = self._new_envelope(payload, kind); sent = self.bus.send_all(msg, self.cfg.uav_id)
        self.tx_packets += sent; return sent
    def send_peer(self, uid, kind, payload):
        if not self.bus or uid == self.cfg.uav_id: return False
        msg = self._new_envelope(payload, kind, dst_uav=uid); ok = self.bus.send_peer(uid, msg)
        if ok: self.tx_packets += 1
        return ok
    def send_gcs(self, kind, payload):
        if not self.bus: return False
        msg = self._new_envelope(payload, kind); msg["dst"] = "GCS"; ok = self.bus.send_gcs(msg)
        if ok: self.tx_packets += 1
        return ok
    def _remember_packet(self, pid):
        if pid in self.seen_packet_set: return False
        if len(self.seen_packets) >= self.seen_packets.maxlen:
            old = self.seen_packets.popleft(); self.seen_packet_set.discard(old)
        self.seen_packets.append(pid); self.seen_packet_set.add(pid); return True

    def receive(self, now):
        if not self.bus: return
        for msg in self.bus.recv_all():
            src = int(msg.get("src_uav", -1))
            if src == self.cfg.uav_id: continue
            self.latencies_ms.append(max(0.0, (time.time() - float(msg.get("sent_ts", time.time()))) * 1000.0))
            self.rx_packets += 1; mtype = str(msg.get("type", ""))
            if mtype == "BEACON": self.receive_beacon(msg, now)
            elif mtype == "STATE": self.receive_beacon(msg, now)
            elif mtype == "TASK_BID": self.receive_task_bid(msg, now)
            elif mtype == "TASK_EVENT": self.receive_task_event(msg, now)
            elif mtype in ("DATA_BURST", "FORWARD_DATA"): self.receive_packet(msg, now)
            elif mtype == "GCS_OVERRIDE": self.handle_gcs_override(msg, now)
            elif mtype == "RECOVERY_REQUEST": self.handle_recovery_request(msg, now)
            elif mtype == "PRIORITY_POI": self.handle_priority_poi(msg, now)

    def receive_beacon(self, msg, now):
        src = int(msg.get("src_uav", -1))
        if src <= 0 or src == self.cfg.uav_id: return
        pos = msg.get("position", {}); vel = msg.get("velocity", {}); prop = msg.get("proposed_direction", {})
        p = Peer(
            uav_id=src, position=Vec3(float(pos.get("x",0)), float(pos.get("y",0)), float(pos.get("z",0))),
            velocity=Vec3(float(vel.get("x",0)), float(vel.get("y",0)), float(vel.get("z",0))),
            battery_pct=float(msg.get("battery_pct", -1)), mode=str(msg.get("mode", "UNKNOWN")),
            healthy=bool(msg.get("healthy", True)), poi_id=msg.get("poi_id"),
            poi_status=str(msg.get("poi_status", PoiStatus.PENDING.value)), poi_priority=int(msg.get("poi_priority", 0)),
            obstacle_distance=float(msg.get("obstacle_distance", float("inf"))),
            proposed_direction=Vec3(float(prop.get("x",0)), float(prop.get("y",0)), float(prop.get("z",0))),
            cluster_id=str(msg.get("cluster_id", f"SOLO-{src}")), last_rx=now,
            sent_ts=float(msg.get("sent_ts", time.time())), tx_seq=int(msg.get("beacon_seq", msg.get("seq", 0))),
            pdr_ewma=float(msg.get("route_pdr", msg.get("pdr_ewma", 1.0))), gcs_direct=bool(msg.get("gcs_direct", False)),
            gcs_hop_count=self._safe_int_or_none(msg.get("gcs_hop_count")),
            gcs_route_quality_db=float(msg.get("gcs_route_quality_db", -999.0)),
            gcs_route_pdr=float(msg.get("gcs_route_pdr", 0.0)),
            neighbor_ids=set(int(x) for x in msg.get("neighbor_ids", []) if int(x) > 0), last_beacon=now
        )
        p.cluster_role = str(msg.get("cluster_role", "member"))
        p.group_role   = str(msg.get("group_role",   "member"))
        peer = self.peers.get(src)
        if peer is not None:
            p.last_distance = peer.last_distance; p.link_quality_db = peer.link_quality_db
            p.connected = peer.connected; p.link_lifetime_s = peer.link_lifetime_s
        self.peers[src] = p; self._update_peer_link(p, now); self.refresh_local_connectivity(now)

    def handle_gcs_override(self, msg, now):
        poi_id = str(msg.get("poi_id", "")); forced_uid = int(msg.get("uav_id", -1))
        if forced_uid == self.cfg.uav_id and poi_id in self.task.pois:
            self.task.gcs_override_uid = forced_uid
            self.task.assigned = poi_id
            self.task.pois[poi_id].assigned_uav = forced_uid
            self.task.pois[poi_id].status = PoiStatus.ASSIGNED
            self.task.vote_start_time = now
            print(f"[GCS] Override accepted: UAV {forced_uid} claims {poi_id}")

    @staticmethod
    def _safe_int_or_none(v):
        try: return int(v) if v is not None else None
        except: return None

    def receive_task_bid(self, msg, now):
        src = int(msg.get("src_uav", -1)); poi_id = str(msg.get("poi_id", ""))
        if src <= 0 or not poi_id: return
        self.task.register_bid(poi_id, src, float(msg.get("bid", float("inf"))), int(msg.get("epoch", 0)), now)
        self.task.recompute_winner(poi_id, now)

    def receive_task_event(self, msg, now):
        poi_id = str(msg.get("poi_id", ""))
        if not poi_id: return
        poi = self.task.pois.get(poi_id)
        if poi is None: return
        try:
            st = PoiStatus(str(msg.get("status", PoiStatus.PENDING.value)))
            poi.status = st
        except: return
        poi.assigned_uav = self._safe_int_or_none(msg.get("assigned_uav"))
        if st == PoiStatus.COMPLETED:
            if self.task.assigned == poi_id or self.locked_poi == poi_id:
                self.task.assigned = None
                self.locked_poi = None
                print(f"[TASK] UAV {self.cfg.uav_id} acknowledged {poi_id} completed by UAV {poi.assigned_uav}. Re-voting...")
        elif poi.assigned_uav != self.cfg.uav_id and self.task.assigned == poi_id:
            if self.locked_poi != poi_id:
                self.task.assigned = None

    def handle_priority_poi(self, msg, now):
        raw = msg.get("poi", {})
        try: poi = Poi(str(raw["poi_id"]), float(raw["x"]), float(raw["y"]), float(raw["z"]), int(raw.get("priority", 10)))
        except: return
        self.task.add_priority_poi(poi); curr = self.task.current()
        # Do not preempt if drone is locked onto an active POI en route
        if self.locked_poi is None and self.task.should_preempt(curr, poi, self.mode, self.bridge_active):
            if curr and curr.poi_id != poi.poi_id:
                self.broadcast_task_event(curr.poi_id, PoiStatus.REASSIGN_REQUIRED.value, None); self.task.release_current()
            self.recovery_event = f"priority interrupt {poi.poi_id}"
        else: self.recovery_event = f"priority queued {poi.poi_id}"

    def handle_recovery_request(self, msg, now):
        if self.mode in (DroneMode.RTH, DroneMode.RETURN_REQUIRED): return
        tgt = msg.get("target", {})
        self.recovery_target = Vec3(float(tgt.get("x",0)), float(tgt.get("y",0)), float(tgt.get("z",0)))
        self.mode = DroneMode.FAULT_RECOVERY; self.bridge_active = True; self.bridge_target = self.recovery_target
        self.recovery_event = f"recovery request from UAV {msg.get('src_uav', '?')}"
        self.recovery_started_at = self.recovery_started_at or now

    def receive_packet(self, msg, now):
        pid = str(msg.get("packet_id", ""))
        if not pid or not self._remember_packet(pid): self.dropped_packets += 1; return
        if int(msg.get("ttl", 0)) <= 0: self.dropped_packets += 1; return
        if str(msg.get("final_dst", "GCS")) != "GCS": return
        self.refresh_local_connectivity(now)
        if self.gcs_direct: self.deliver_to_gcs(msg, now); return
        self.forward_packet(msg, now)

    def deliver_to_gcs(self, msg, now):
        self.delivered_to_gcs += 1
        self.latencies_ms.append(max(0.0, (time.time() - float(msg.get("created_ts", msg.get("sent_ts", time.time())))) * 1000.0))
        if self.bus:
            sink = dict(msg); sink["type"] = "GCS_DELIVERY"; sink["delivered_by_uav"] = self.cfg.uav_id
            self.bus.send_gcs(sink)

    def forward_packet(self, msg, now):
        ttl = int(msg.get("ttl", 0))
        if ttl <= 1: self.dropped_packets += 1; return False
        nh = self.next_hop(now)
        if nh is None: self._queue_packet(DataPacket(str(msg.get("packet_id")), int(msg.get("origin_uav",-1)), "GCS", dict(msg.get("payload",{})), ttl, [int(x) for x in msg.get("path",[])], float(msg.get("created_ts",time.time())), now, int(msg.get("attempts",0))+1)); self.recovery_event="route unavailable"; return False
        path = [int(x) for x in msg.get("path", [])]
        if self.cfg.uav_id not in path: path.append(self.cfg.uav_id)
        if nh in path: self.dropped_packets += 1; return False
        fwd = dict(msg); fwd["type"]="FORWARD_DATA"; fwd["src_uav"]=self.cfg.uav_id; fwd["dst_uav"]=nh
        fwd["ttl"]=ttl-1; fwd["path"]=path; fwd["forwarded_ts"]=time.time(); fwd["attempts"]=int(msg.get("attempts",0))+1
        if self.send_peer(nh, "FORWARD_DATA", fwd): self.forwarded_packets += 1; self.next_hop_id = nh; return True
        self._queue_packet(DataPacket(str(msg.get("packet_id")), int(msg.get("origin_uav",-1)), "GCS", dict(msg.get("payload",{})), ttl-1, path, float(msg.get("created_ts",time.time())), now, int(msg.get("attempts",0))+1))
        return False

    def _queue_packet(self, p):
        if len(self.forward_queue) >= self.cfg.max_forward_queue: self.forward_queue.popleft(); self.dropped_packets += 1
        self.forward_queue.append(p)
    def flush_forward_queue(self, now):
        if not self.forward_queue or now - self.forward_queue[0].last_try_ts < self.cfg.packet_retry_period_s: return
        p = self.forward_queue.popleft(); p.last_try_ts = now; p.attempts += 1; nh = self.next_hop(now)
        if nh is None: self.forward_queue.append(p); return
        if p.ttl <= 1: self.dropped_packets += 1; return
        msg = p.to_dict(); msg["path"] = list(p.path) + ([self.cfg.uav_id] if self.cfg.uav_id not in p.path else [])
        msg["ttl"] = p.ttl - 1; msg["type"] = "FORWARD_DATA"; msg["dst_uav"] = nh
        if self.send_peer(nh, "FORWARD_DATA", msg): self.forwarded_packets += 1
        else: self.forward_queue.append(p)

    def _update_peer_link(self, p, now):
        d = dist(self.t.position, p.position); old_d = p.last_distance; old_t = self.prev_peer_time.get(p.uav_id, now)
        dt = max(1e-3, now - old_t)
        old_q = p.link_quality_db
        p.last_distance = d; p.link_quality_db = self.link.quality(d); p.connected = self.link.connected(d)
        p.pdr_ewma = (1.0 - self.cfg.pdr_alpha) * p.pdr_ewma + self.cfg.pdr_alpha * self.link.pdr_estimate(d)
        cs = (old_d - d) / dt if old_d > 0 else 0.0
        if cs > 0.05:
            rem = max(0.0, d - self.cfg.r_safe_m)
            p.link_lifetime_s = rem / cs if rem > 0 else 0.0
        else: p.link_lifetime_s = float("inf")
        # Track signal degradation rate (dB/s).  Positive means link is getting WORSE.
        raw_rate = (old_q - p.link_quality_db) / dt  # positive = degrading
        p.signal_rate_db_s = raw_rate
        alpha_deg = 0.30  # EWMA weight — smooth, but responsive
        p.degradation_rate = (1.0 - alpha_deg) * p.degradation_rate + alpha_deg * max(0.0, raw_rate)
        self.prev_peer_dist[p.uav_id] = d; self.prev_peer_time[p.uav_id] = now

    def _local_direct_gcs_link(self): return self.link.connected(dist(self.t.position, self.cfg.gcs_position))
    def refresh_local_connectivity(self, now):
        self.gcs_direct = self._local_direct_gcs_link()
        stale = [uid for uid, p in self.peers.items() if now - p.last_rx > self.cfg.peer_timeout_s]
        for uid in stale:
            self.peers.pop(uid, None); self.neighbor_hops.pop(uid, None); self.prev_peer_dist.pop(uid, None); self.prev_peer_time.pop(uid, None)
        for p in self.peers.values(): self._update_peer_link(p, now)
        fresh_hops = {}
        if self.gcs_direct: local_hop = 0
        else:
            local_hop = None
            for p in self.peers.values():
                if not p.connected or not p.healthy: continue
                cand = 1 if p.gcs_direct else (p.gcs_hop_count + 1 if p.gcs_hop_count is not None else None)
                if cand is not None and cand <= self.cfg.route_max_hops:
                    fresh_hops[p.uav_id] = cand
                    local_hop = cand if local_hop is None else min(local_hop, cand)
        self.neighbor_hops = fresh_hops; self.gcs_hop_count = local_hop
        if self.gcs_direct:
            dd = dist(self.t.position, self.cfg.gcs_position)
            self.gcs_route_quality_db = self.link.quality(dd); self.gcs_route_pdr = self.link.pdr_estimate(dd)
            self.route_entry = RouteEntry(None, 0, self.gcs_route_quality_db, self.gcs_route_pdr, float("inf"), now); self.next_hop_id = None
        else:
            cand = self._best_route_candidate(now)
            if cand:
                p, _ = cand; self.next_hop_id = p.uav_id
                self.gcs_hop_count = (p.gcs_hop_count + 1) if p.gcs_hop_count is not None else 1
                arq = p.gcs_route_quality_db
                if p.gcs_direct and arq <= -998.0: arq = p.link_quality_db
                arp = p.gcs_route_pdr if p.gcs_route_pdr > 0 else (1.0 if p.gcs_direct else 0.0)
                self.gcs_route_quality_db = min(p.link_quality_db, arq); self.gcs_route_pdr = min(p.pdr_ewma, arp)
                self.route_entry = RouteEntry(p.uav_id, self.gcs_hop_count, self.gcs_route_quality_db, self.gcs_route_pdr, p.link_lifetime_s, now)
            else:
                self.next_hop_id = None; self.gcs_hop_count = None; self.gcs_route_quality_db = -999.0; self.gcs_route_pdr = 0.0; self.route_entry = None
        if not self.gcs_direct and self.next_hop_id is None: self.link_at_risk = True

    def _best_route_candidate(self, now):
        cands = []; c_dist = dist(self.t.position, self.cfg.gcs_position)
        for p in self.peers.values():
            if not p.connected or not p.healthy: continue
            hop = 0 if p.gcs_direct else p.gcs_hop_count
            if hop is None or hop + 1 > self.cfg.route_max_hops: continue
            pdr = clamp01(p.pdr_ewma * max(0.0, p.gcs_route_pdr if p.gcs_route_pdr > 0 else 1.0))
            ln = clamp01((p.link_quality_db - self.cfg.link_margin_db) / 25.0)
            lt = 1.0 if math.isinf(p.link_lifetime_s) else clamp01(p.link_lifetime_s / 10.0)
            prog = clamp01((c_dist - dist(p.position, self.cfg.gcs_position)) / c_dist) if c_dist > 1e-6 else 0.0
            batt = safe_battery(p.battery_pct); hs = clamp01(1.0 - hop / max(1, self.cfg.route_max_hops))
            sc = self.cfg.routing_w_link*ln + self.cfg.routing_w_pdr*pdr + self.cfg.routing_w_progress*prog + self.cfg.routing_w_lifetime*lt + self.cfg.routing_w_battery*batt + self.cfg.routing_w_hops*hs
            cands.append((p, sc))
        if not cands: return None
        return max(cands, key=lambda i: (i[1], -i[0].uav_id))

    def next_hop(self, now=None):
        now = time.monotonic() if now is None else now; self.refresh_local_connectivity(now)
        if self.gcs_direct: return None
        return self.next_hop_id

    def broadcast_beacon(self, obs_d, proposed, now):
        if not self.bus or now - self.last_beacon < 1.0 / max(self.cfg.control_hz, 1.0): return
        self.last_beacon = now; self.beacon_seq += 1; self.refresh_local_connectivity(now)
        p = self.task.current(); cids = [x.uav_id for x in self.peers.values() if x.connected and x.healthy]
        self.broadcast("BEACON", {
            "beacon_seq": self.beacon_seq, "position": asdict(self.t.position), "velocity": asdict(self.t.velocity),
            "battery_pct": self.t.battery_pct, "healthy": self.t.healthy, "mode": self.mode.value,
            "poi_id": self.task.assigned, "poi_status": p.status.value if p else PoiStatus.PENDING.value,
            "poi_priority": p.priority if p else 0, "obstacle_distance": obs_d,
            "proposed_direction": asdict(proposed), "cluster_id": self.cluster_id,
            "formation_radius_m": self.cfg.formation_radius_m, "gcs_direct": self.gcs_direct,
            "gcs_hop_count": self.gcs_hop_count, "gcs_route_quality_db": self.gcs_route_quality_db,
            "gcs_route_pdr": self.gcs_route_pdr, "route_pdr": self.gcs_route_pdr,
            "neighbor_ids": cids, "bridge_active": self.bridge_active,
            "cluster_role": self.cluster_role, "group_role": self.group_role
        })

    def broadcast_bids(self, now):
        if not self.bus or self.locked_poi or self.task.assigned or now - self.last_bid < self.cfg.bid_period_s: return
        self.last_bid = now; conn = sum(1 for p in self.peers.values() if p.connected); prog = self.task.progress_fraction(self.t.position)
        for poi in self.task.pois.values():
            if poi.status not in (PoiStatus.PENDING, PoiStatus.REASSIGN_REQUIRED, PoiStatus.DEFERRED): continue
            bid = self.task.score(poi, self.t.position, self.t.battery_pct, conn, prog)
            self.task.register_bid(poi.poi_id, self.cfg.uav_id, bid, self.task.local_epoch, now)
            self.broadcast("TASK_BID", {
                "poi_id": poi.poi_id, "priority": poi.priority, "bid": bid, "epoch": self.task.local_epoch,
                "winner_uav": self.task.winner_table.get(poi.poi_id).uav_id if poi.poi_id in self.task.winner_table else None
            })

    def broadcast_task_event(self, poi_id, status, assigned_uav):
        self.broadcast("TASK_EVENT", {"poi_id": poi_id, "status": status, "assigned_uav": assigned_uav, "epoch": self.task.local_epoch})

    def maybe_assign_task(self, now):
        if self.locked_poi or self.task.assigned: return
        conn = sum(1 for p in self.peers.values() if p.connected)
        chosen = self.task.local_try_assign(self.cfg.uav_id, self.t.position, self.t.battery_pct, conn, now, self.mode, self.bridge_active)
        if chosen:
            self.locked_poi = chosen.poi_id
            self.broadcast_task_event(chosen.poi_id, chosen.status.value, self.cfg.uav_id)

    def usable_formation_peers(self): return [p for p in self.peers.values() if p.healthy and time.monotonic()-p.last_rx <= self.cfg.peer_timeout_s and p.connected]
    def formation_center_estimate(self): return mean_vec([self.t.position] + [p.position for p in self.usable_formation_peers()])
    
    def connectivity_correction(self):
        corr = Vec3()
        for p in self.peers.values():
            if not p.healthy: continue
            d = dist(self.t.position, p.position); dir_p = (p.position - self.t.position).hnormalized()
            if dir_p.norm() < 1e-9: continue
            if p.connected and d > self.cfg.r_safe_m - self.cfg.connectivity_target_margin_m:
                corr += dir_p * max(0.0, d - (self.cfg.r_safe_m - self.cfg.connectivity_target_margin_m)) * self.cfg.connectivity_gain
            if not p.connected and d <= self.cfg.r_max_m * 1.20:
                corr += dir_p * max(0.0, d - (self.cfg.r_safe_m * 0.85)) * (self.cfg.connectivity_gain * 0.55)
            if p.gcs_direct or p.gcs_hop_count is not None:
                rs = max(0.0, min(1.0, p.gcs_route_pdr if p.gcs_route_pdr > 0 else 0.5))
                corr += dir_p * rs * 0.25
        if not self.gcs_direct and self.next_hop_id is not None:
            p = self.peers.get(self.next_hop_id)
            if p is not None: corr += (p.position - self.t.position).hnormalized() * 0.5

        # --- Singleton force-merge: re-converge toward swarm if fully isolated ---
        if not self.peers:
            now = time.monotonic()
            isolated_s = (now - self.disconnect_started_at) if self.disconnect_started_at is not None else 0.0
            isolation_threshold = self.cfg.peer_timeout_s * 2.0
            if isolated_s >= isolation_threshold:
                # Pull toward last known relay position if we remember it
                last_relay_peer = None  # relay may have already timed out of peers dict
                if self.last_relay is not None:
                    last_relay_peer = self.peers.get(self.last_relay)  # will be None (already expired)
                # Fallback: pull toward GCS as a beacon — the GCS is always reachable from the swarm core
                gcs_dir = (self.cfg.gcs_position - self.t.position).hnormalized()
                if gcs_dir.norm() > 1e-9:
                    # Scale force with isolation time: stronger pull the longer we've been alone
                    force_mag = min(self.cfg.network_deform_limit_m, (isolated_s - isolation_threshold) * 0.5 + 1.0)
                    corr += gcs_dir * force_mag

        return corr.clamp(self.cfg.network_deform_limit_m)


    def obstacle_vote(self, gdir, own_obs, own_prop):
        # Prioritize local obstacle avoidance if dangerously close
        if own_obs < 9.0 and own_prop.norm() > 1e-9:
            return own_prop.normalized()
        votes = [{"uav_id": self.cfg.uav_id, "weight": 1.0/max(0.25, finite_or(own_obs, 1e9)), "obstacle_distance": finite_or(own_obs, float("inf")), "direction": own_prop.normalized()}]
        for p in self.peers.values():
            if p.cluster_id != self.cluster_id or not p.healthy or not math.isfinite(p.obstacle_distance) or p.obstacle_distance > 12.0: continue
            votes.append({"uav_id": p.uav_id, "weight": 1.0/max(0.25, p.obstacle_distance), "obstacle_distance": p.obstacle_distance, "direction": p.proposed_direction.normalized()})
        if not votes: return gdir.normalized()
        tw = sum(v["weight"] for v in votes)
        if tw <= 1e-9: return gdir.normalized()
        agg = Vec3()
        for v in votes: agg += v["direction"] * (v["weight"] / tw)
        res = agg.normalized()
        if res.norm() < 1e-9:
            c = min(votes, key=lambda v: v["obstacle_distance"])
            return c["direction"] if c["direction"].norm() > 1e-9 else gdir.normalized()
        return res

    def predictive_breach(self, now):
        risk = False; dt_nom = 1.0 / max(self.cfg.control_hz, 0.1)
        for p in self.peers.values():
            nd = dist(self.t.position, p.position); pd = self.prev_peer_dist.get(p.uav_id, nd); pt = self.prev_peer_time.get(p.uav_id, now - dt_nom)
            dt = max(1e-3, now - pt); closing = (pd - nd) / dt
            if closing > 0 and nd > self.cfg.r_safe_m:
                if (nd - self.cfg.r_safe_m) / closing < self.cfg.reaction_threshold_s: risk = True
            pa = self.t.position + self.t.velocity * self.cfg.prediction_horizon_s
            pb = p.position + p.velocity * self.cfg.prediction_horizon_s
            if dist(pa, pb) < self.cfg.r_safe_m: risk = True
            self.prev_peer_dist[p.uav_id] = nd; self.prev_peer_time[p.uav_id] = now
        self.link_at_risk |= risk; return risk

    def collision_avoid(self, target):
        corr = Vec3(); viol = False; near = float("inf"); min_sep = float("inf")
        emergency_avoidance = False
        ps = self.t.position + self.t.velocity * self.cfg.prediction_horizon_s
        avoidance_separation = max(8.0, self.cfg.min_separation_m * self.cfg.collision_anticipation_factor)
        for p in self.peers.values():
            if not p.healthy: continue
            cd = dist(self.t.position, p.position); pp = p.position + p.velocity * self.cfg.prediction_horizon_s
            pd = dist(ps, pp); near = min(near, cd); min_sep = min(min_sep, cd)
            if cd < self.cfg.min_separation_m:
                viol = True
            if cd < self.cfg.min_separation_m or pd < (self.cfg.min_separation_m * 0.8):
                emergency_avoidance = True
            if cd < avoidance_separation or pd < avoidance_separation:
                away = (self.t.position - p.position).hnormalized()
                if away.norm() < 1e-9:
                    away = Vec3(1 if self.cfg.uav_id < p.uav_id else -1, 0, 0)
                strg = max(0.0, avoidance_separation - min(cd, pd))
                corr += away * strg * self.cfg.avoidance_gain * self.cfg.collision_avoidance_multiplier
        if emergency_avoidance and corr.norm() > 1e-9:
            escape = corr.normalized() * min(self.cfg.max_target_step_m, corr.norm())
            target = self.t.position + escape
        else:
            target += corr.clamp(self.cfg.max_avoidance_offset_m)
        # Never let collision avoidance change altitude — Z is PX4's domain
        target.z = -abs(self.cfg.cruise_altitude_m)
        return target, viol, near, min_sep

    def adaptive_spacing_force(self, peers):
        if not peers: return Vec3()
        f = Vec3(); rad = max(self.cfg.min_separation_m * 1.5, self.cfg.formation_radius_m)
        des = max(self.cfg.min_separation_m * 1.25, rad * 0.85)
        des = min(des, self.cfg.r_safe_m * 0.45 if self.cfg.r_safe_m > 0 else des)
        for p in peers:
            d = max(0.1, dist(self.t.position, p.position)); dir_p = (p.position - self.t.position).hnormalized()
            f += dir_p * (-(d - des) / max(des, 1.0))
        return f * self.cfg.adaptive_spacing_gain

    def fluid_formation_target(self, direction):
        peers = self.usable_formation_peers(); center = self.formation_center_estimate()
        mf = direction.hnormalized() * self.cfg.mission_attraction_gain
        ce = center - self.t.position; rd = ce.hnorm()
        outer = max(self.cfg.min_separation_m * 2.0, self.cfg.formation_radius_m * self.cfg.fluid_outer_radius_factor)
        inner = max(self.cfg.min_separation_m * 0.90, self.cfg.formation_radius_m * self.cfg.fluid_inner_radius_factor)
        coh = Vec3()
        if rd > outer: coh = ce.hnormalized() * (rd - outer)
        elif rd < inner and rd > 1e-6: coh = -ce.hnormalized() * (inner - rd)
        coh *= self.cfg.fluid_cohesion_gain
        sep = Vec3()
        for p in peers:
            d = dist(self.t.position, p.position)
            if d < self.cfg.min_separation_m * 2.5:
                away = (self.t.position - p.position).hnormalized()
                if away.norm() < 1e-9: away = Vec3(1,0,0)
                sep += away * clamp01((self.cfg.min_separation_m * 2.5 - d) / max(self.cfg.min_separation_m, 0.1))
        sep *= self.cfg.fluid_separation_gain
        spc = self.adaptive_spacing_force(peers); alg = Vec3()
        if peers: alg = (mean_vec([p.velocity for p in peers]) - self.t.velocity) * self.cfg.fluid_alignment_gain
        con = self.connectivity_correction()
        f = (mf + coh + sep + spc + alg + con).clamp(self.cfg.network_deform_limit_m)
        # Only apply cruise step when there is an actual mission direction; zero direction means hold-position
        if direction.hnorm() > 1e-6:
            bs = direction.hnormalized() * self.cfg.cruise_speed_mps / max(self.cfg.control_hz, 1.0)
        else:
            bs = Vec3()  # no cruise drift when idle/holding
        tgt = self.t.position + bs + f * (1.0 / max(self.cfg.control_hz, 1.0))
        tgt.z = -abs(self.cfg.cruise_altitude_m)
        ds = [dist(self.t.position, p.position) for p in peers]
        self.formation_error = abs(sum(ds)/len(ds) - min(outer, self.cfg.r_safe_m * 0.45)) if ds else 0.0
        self.network_deformation = con.norm()
        return tgt

    def local_graph(self):
        g = {self.cfg.uav_id: set()}
        for p in self.peers.values():
            if not p.healthy: continue
            g.setdefault(p.uav_id, set())
            if p.connected: g[self.cfg.uav_id].add(p.uav_id); g[p.uav_id].add(self.cfg.uav_id)
            for n in p.neighbor_ids:
                if n <= 0 or n == p.uav_id: continue
                g.setdefault(n, set()); g[p.uav_id].add(n); g[n].add(p.uav_id)
        return g

    def topology_criticality(self, uid):
        g = self.local_graph()
        if uid not in g: return 0.0
        def reach(rem):
            st = self.cfg.uav_id
            if st == rem: return 0
            sn = {st}; q = deque([st])
            while q:
                c = q.popleft()
                for nx in g.get(c, set()):
                    if nx == rem or nx in sn: continue
                    sn.add(nx); q.append(nx)
            return len(sn)
        full = reach(None); red = reach(uid)
        return clamp01((full - red) / max(1, full - 1))

    # ------------------------------------------------------------------
    # 3-Tier Hierarchy: Member → Cluster-Head (CH) → Group-Head (GCH)
    # Each cluster = cfg.cluster_size drones with best-link drone as CH.
    # Each group = cfg.group_size clusters with GCS-closest CH as GCH.
    # GCH coordinates inter-cluster voting and task pre-planning.
    # ------------------------------------------------------------------
    def elect_hierarchy(self):
        peers = [p for p in self.peers.values() if p.connected and p.healthy]
        # Cluster: take nearest cluster_size-1 peers by link quality
        cluster_peers = sorted(peers, key=lambda p: -p.link_quality_db)[:self.cfg.cluster_size - 1]
        my_lq = self.gcs_route_quality_db if self.gcs_route_quality_db > -900 else -999.0
        i_am_ch = all(my_lq >= p.link_quality_db for p in cluster_peers)
        # Stable tie-break: lower uav_id wins when scores are equal
        if i_am_ch and cluster_peers:
            i_am_ch = all(
                my_lq > p.link_quality_db or self.cfg.uav_id < p.uav_id
                for p in cluster_peers
            )
        self.cluster_role = "ch" if i_am_ch else "member"
        # Group: among CHs, best GCS link quality wins
        ch_peers = [p for p in peers if p.cluster_role == "ch"]
        if self.cluster_role == "ch":
            my_gcs = self.gcs_route_quality_db or -999.0
            i_am_gch = all(my_gcs >= (p.gcs_route_quality_db or -999.0) for p in ch_peers)
            if i_am_gch and ch_peers:
                i_am_gch = all(
                    my_gcs > (p.gcs_route_quality_db or -999.0) or self.cfg.uav_id < p.uav_id
                    for p in ch_peers
                )
            self.group_role = "gch" if i_am_gch else "ch"
        else:
            self.group_role = "member"

    def relay_score(self, p):
        ls = clamp01((p.link_quality_db - self.cfg.link_margin_db) / 25.0)
        bs = safe_battery(p.battery_pct); ts = self.topology_criticality(p.uav_id)
        rs = clamp01(p.gcs_route_pdr if p.gcs_route_pdr > 0 else (1.0 if p.gcs_direct else 0.0))
        tc = clamp01(p.poi_priority / 5.0) if p.poi_status in (PoiStatus.ASSIGNED.value, PoiStatus.IN_PROGRESS.value) else 0.0
        ta = 1.0 - tc
        # Degradation penalty: heavily penalise links that are degrading quickly (>2 dB/s)
        deg_penalty = clamp01(p.degradation_rate / 10.0) * 0.15
        return self.cfg.relay_link_weight*ls + self.cfg.relay_battery_weight*bs + self.cfg.relay_topology_weight*ts + self.cfg.relay_task_weight*ta + self.cfg.relay_route_weight*rs - deg_penalty


    def choose_relay(self):
        now = time.monotonic()
        cands = [p for p in self.peers.values() if p.connected and p.healthy]
        if not cands:
            if self.last_relay is not None: self.relay_changes += 1
            self.last_relay = None; self.relay_candidate_id = None
            return None

        best = max(cands, key=lambda p: (self.relay_score(p), -p.uav_id))
        best_uid = best.uav_id

        # --- Emergency bypass: current relay is critical, switch immediately ---
        cur = self.peers.get(self.last_relay) if self.last_relay else None
        critical_floor = self.cfg.rx_sensitivity_dbm + 5.0
        emergency = cur is not None and cur.link_quality_db < critical_floor

        if emergency or self.last_relay is None:
            # No hysteresis needed: immediate switch
            if best_uid != self.last_relay:
                self.relay_changes += 1; self.last_relay_switch_ts = now
            self.last_relay = best_uid; self.relay_candidate_id = None; self.relay_candidate_since = 0.0
            return best

        # --- Adaptive hysteresis hold time ---
        # Faster switch when Fiedler λ2 is low (fragile network) or relay degrades fast.
        lambda2 = self.last_lambda2 if self.last_lambda2 > 0 else 1.0
        deg = cur.degradation_rate if cur else 0.0
        # Shorter hold when network is fragile or link is actively dying
        adaptive_hold = max(0.5, self.cfg.hysteresis_hold_s / max(0.5, lambda2) - deg * 0.25)
        adaptive_hold = min(adaptive_hold, self.cfg.hysteresis_hold_s)  # never exceed config ceiling

        if best_uid == self.last_relay:
            # Incumbent still best — keep candidate tracking reset
            self.relay_candidate_id = None; self.relay_candidate_since = 0.0
            return best

        # New candidate must beat current by hysteresis_margin_db AND hold for adaptive_hold
        cur_score = self.relay_score(cur) if cur else -1.0
        best_score = self.relay_score(best)
        margin_ok = (best_score - cur_score) >= (self.cfg.hysteresis_margin_db / 25.0)  # normalise to score units

        if margin_ok:
            if self.relay_candidate_id != best_uid:
                self.relay_candidate_id = best_uid; self.relay_candidate_since = now
            elif now - self.relay_candidate_since >= adaptive_hold:
                # Candidate has held its lead long enough — commit the switch
                self.relay_changes += 1; self.last_relay_switch_ts = now
                self.last_relay = best_uid; self.relay_candidate_id = None; self.relay_candidate_since = 0.0
                return best
        else:
            # Candidate no longer ahead by margin — reset
            self.relay_candidate_id = None; self.relay_candidate_since = 0.0

        # Return incumbent (or best if incumbent gone)
        result = self.peers.get(self.last_relay)
        if result is None or not result.connected:
            self.last_relay = best_uid; self.relay_changes += 1; self.last_relay_switch_ts = now
            return best
        return result


    def handle_data(self, now):
        if not self.bus or now - self.last_burst < 2.0: return
        self.last_burst = now; self.refresh_local_connectivity(now)
        pid = f"uav{self.cfg.uav_id}-{self.data_seq}-{int(now * 1000)}"; self.data_seq += 1
        p = self.task.current()
        
        gcs_dist = dist(self.t.position, self.cfg.gcs_position)
        sig_db = self.link.rx_power(gcs_dist)
        
        graph = self.local_graph(); nodes = list(graph.keys())
        if np is not None and len(nodes) > 1:
            adj = np.zeros((len(nodes), len(nodes)))
            for i, u in enumerate(nodes):
                for j, v in enumerate(nodes):
                    if v in graph[u]: adj[i, j] = 1.0
            deg = np.diag(adj.sum(axis=1)); lap = deg - adj; eigvals = np.linalg.eigvalsh(lap)
            lambda2 = float(eigvals[1]) if len(eigvals) > 1 else 0.0
        else: lambda2 = 0.0
        self.last_lambda2 = lambda2  # persisted for adaptive hysteresis in choose_relay()

        r = self.cfg.r_safe_m
        polygon_3d = []
        for i in range(12):
            angle = 2 * math.pi * i / 12.0
            px = self.t.position.x + r * math.cos(angle)
            py = self.t.position.y + r * math.sin(angle)
            pz = self.t.position.z
            polygon_3d.append([px, py, pz])

        connected_ids = [p.uav_id for p in self.peers.values() if p.connected and p.healthy]

        payload = {
            "position": asdict(self.t.position), "battery_pct": self.t.battery_pct, "mode": self.mode.value,
            "poi_id": self.task.assigned, "cluster_id": self.cluster_id, "uav_id": self.cfg.uav_id,
            "signal_strength_db": sig_db, "lambda2_connectivity": lambda2,
            "obstacle_distance": self.obstacle.update(self.t.position, Vec3(), self.t.tof_distance_m)[0],
            "polygon_3d": polygon_3d,
            "neighbor_ids": connected_ids
        }
        pkt = {"packet_id": pid, "origin_uav": self.cfg.uav_id, "final_dst": "GCS", "payload": payload,
               "ttl": self.cfg.packet_ttl, "path": [self.cfg.uav_id], "created_ts": time.time()}
        self._remember_packet(pid)
        if self.gcs_direct:
            pkt["type"] = "DATA_BURST"
            if self.send_gcs("DATA_BURST", pkt): self.delivered_to_gcs += 1
            else: self._queue_packet(DataPacket(pid, self.cfg.uav_id, "GCS", payload, self.cfg.packet_ttl, [self.cfg.uav_id], pkt["created_ts"]))
            return
        hop = self.next_hop(now)
        if hop is not None and self.send_peer(hop, "DATA_BURST", pkt): self.next_hop_id = hop; return
        self._queue_packet(DataPacket(pid, self.cfg.uav_id, "GCS", payload, self.cfg.packet_ttl, [self.cfg.uav_id], pkt["created_ts"])); self.link_at_risk = True

    def plan_bridge_chain(self, start, end):
        gap = dist(start, end)
        if gap <= self.cfg.r_safe_m: return []
        hn = int(math.ceil(gap / max(self.cfg.r_safe_m, 1e-6))); bc = max(0, hn - 1)
        if bc == 0: return []
        return [start + (end - start) * (i / (bc + 1)) for i in range(1, bc + 1)]

    def request_recovery(self, now):
        cands = [p for p in self.peers.values() if p.healthy and p.battery_pct != 0 and now - p.last_rx <= self.cfg.peer_timeout_s]
        if not cands: self.recovery_event = "no recovery candidate"; return
        def cs(p):
            crit = p.poi_priority if p.poi_id else 0
            tp = 100.0 if p.poi_status == PoiStatus.IN_PROGRESS.value and crit >= 4 else 0.0
            return dist(self.t.position, p.position) + 20.0*clamp01(crit/5.0) + tp - 10.0*safe_battery(p.battery_pct) - 10.0*self.topology_criticality(p.uav_id)
        cand = min(cands, key=cs); mp = (self.t.position + cand.position) * 0.5
        self.send_peer(cand.uav_id, "RECOVERY_REQUEST", {"target": asdict(mp), "reason": "recovery", "requester": self.cfg.uav_id})
        self.recovery_event = f"requested recovery from UAV {cand.uav_id}"
        self.recovery_started_at = self.recovery_started_at or now; self.recovery_count = max(1, self.recovery_count)

    def check_recovery(self, now):
        self.refresh_local_connectivity(now); gr = self.gcs_direct or self.gcs_hop_count is not None
        if gr and self.mode == DroneMode.FAULT_RECOVERY:
            if self.recovery_started_at is not None: self.recovery_time_last_s = now - self.recovery_started_at
            self.recovery_started_at = None; self.mode = DroneMode.NORMAL if self.t.battery_pct < 0 or self.t.battery_pct > self.cfg.low_battery_pct else DroneMode.LOW_BATTERY_WARNING
            self.bridge_active = False; self.bridge_target = None; self.recovery_target = None
            self.recovery_event = "GCS connectivity restored"; self.link_at_risk = False; return
        if self.link_at_risk and not gr:
            if self.mode != DroneMode.FAULT_RECOVERY:
                self.mode = DroneMode.FAULT_RECOVERY; self.recovery_count += 1; self.recovery_started_at = self.recovery_started_at or now
            if now - (self.recovery_started_at or now) <= self.cfg.recovery_timeout_s: self.request_recovery(now)

    def geofence(self, target):
        # Strict hard geofence boundary with 10m safety buffer for 100x100 area
        clamped_x = min(max(target.x, 10.0), 90.0)
        clamped_y = min(max(target.y, 10.0), 90.0)
        clamped_z = -abs(self.cfg.cruise_altitude_m)
        # Real violation is if CURRENT drone position is out of the 100x100 boundary (< 0 or > 100)
        viol = (self.t.position.x < 0.0 or self.t.position.x > 100.0 or 
                self.t.position.y < 0.0 or self.t.position.y > 100.0)
        return Vec3(clamped_x, clamped_y, clamped_z), viol

    def battery_policy(self):
        b = self.t.battery_pct
        if b < 0: return
        if b <= self.cfg.rth_battery_pct and self.mode != DroneMode.RTH:
            self.mode = DroneMode.RETURN_REQUIRED; self.rth_triggered = True
            old = self.task.release_current()
            if old: self.broadcast_task_event(old, PoiStatus.REASSIGN_REQUIRED.value, None)
            self.recovery_event = "RTH battery threshold"
        elif b <= self.cfg.low_battery_pct and self.mode == DroneMode.NORMAL: self.mode = DroneMode.LOW_BATTERY_WARNING

    def cycle(self):
        now = time.monotonic(); self.total_cycles += 1
        if self.ap: self.ap.pump(self.t)
        self.receive(now); self.refresh_local_connectivity(now)
        gr = self.gcs_direct or self.gcs_hop_count is not None
        if gr:
            if self.disconnect_started_at is not None: self.connectivity_downtime_s += now - self.disconnect_started_at; self.disconnect_started_at = None
        elif self.disconnect_started_at is None: self.disconnect_started_at = now
        
        self.elect_hierarchy()
        self.maybe_assign_task(now)
        comp = self.task.mark_progress(self.t.position, self.cfg.uav_id)
        if comp:
            self.broadcast_task_event(comp, PoiStatus.COMPLETED.value, self.cfg.uav_id)
            self.last_completed_poi_pos = Vec3(self.t.position.x, self.t.position.y, self.t.position.z)
            self.locked_poi = None
            self.task.assigned = None
            print(f"[TASK] UAV {self.cfg.uav_id} reached and completed {comp}!")

        # Dynamic swarm coordination: check remaining mission POIs
        all_pois = list(self.task.pois.values())
        uncompleted_pois = [p for p in all_pois if p.status != PoiStatus.COMPLETED]
        pending_pois = [p for p in uncompleted_pois if p.status in (PoiStatus.PENDING, PoiStatus.REASSIGN_REQUIRED, PoiStatus.DEFERRED)]
        active_pois = [p for p in uncompleted_pois if p.status in (PoiStatus.ASSIGNED, PoiStatus.IN_PROGRESS)]

        if not uncompleted_pois:
            self.mode = DroneMode.COMPLETE
        elif self.locked_poi is None:
            # Re-vote if there are pending (unassigned) POIs to claim
            if pending_pois:
                self.maybe_assign_task(now)
            # If all remaining POIs are already handled by other drones, hold position
            # (global_direction() returns direction to last_completed_poi_pos or field center)

        cluster_num = (self.cfg.uav_id - 1) // max(1, self.cfg.cluster_size) + 1
        self.cluster_id = f"CLUSTER-{cluster_num}"
        gdir = self.global_direction()
        od, ood = self.obstacle.update(self.t.position, gdir, self.t.tof_distance_m)
        vd = self.obstacle_vote(gdir, od, ood)
        self.predictive_breach(now)
        
        if self.mode == DroneMode.FAULT_RECOVERY and self.recovery_target is not None: tgt = self.recovery_target
        elif self.bridge_active and self.bridge_target is not None: tgt = self.bridge_target
        else: tgt = self.fluid_formation_target(vd)
        
        # Immediate obstacle repulsion only (pure XY, geofence handles Z)
        if od < 7.5 and ood.norm() > 1e-9:
            push = ood.hnormalized() * min(self.cfg.max_target_step_m, (7.5 - od) * 1.2)
            tgt.x += push.x; tgt.y += push.y

        tgt, self.collision_violation, near, min_sep = self.collision_avoid(tgt)
        tgt, self.geofence_violation = self.geofence(tgt)
        self.nearest_neighbor = near; self.battery_policy()
        
        if self.mode == DroneMode.RETURN_REQUIRED:
            self.mode = DroneMode.RTH
            if self.ap: self.ap.rtl()
            
        self.check_recovery(now); self.flush_forward_queue(now)
        
        if self.ap and self.mode not in (DroneMode.RTH, DroneMode.COMPLETE):
            step = tgt - self.t.position
            step_cap = self.cfg.max_target_step_m * self.speed_scale
            if step.norm() > step_cap: tgt = self.t.position + step.normalized() * step_cap
            tgt, _ = self.geofence(tgt)   # final hard clamp with correct Z
            self.ap.position_target(tgt); self.ap.ensure_offboard()
        elif self.dry_run and self.mode not in (DroneMode.RTH, DroneMode.COMPLETE):
            step = tgt - self.t.position; dt = 1.0 / max(self.cfg.control_hz, 1.0)
            step_cap = self.cfg.max_target_step_m * self.speed_scale
            if step.norm() > step_cap: tgt = self.t.position + step.normalized() * step_cap
            tgt, _ = self.geofence(tgt)
            self.t.velocity = (tgt - self.t.position) * (1.0 / max(dt, 1e-6)); self.t.position = tgt

            
        self.target = tgt; self.broadcast_beacon(od, ood, now); self.broadcast_bids(now)
        self.handle_data(now)
        # Update speed_scale: throttle during relay re-election, full speed when stable
        self.speed_scale = 0.5 if self.relay_candidate_id is not None else 1.0
        r = self.choose_relay(); rs = self.relay_score(r) if r else 0.0
        self.log_cycle(od, rs, min_sep)

    def global_direction(self):
        c = self.task.current()
        if c is not None:
            return (c.pos - self.t.position).hnormalized()
        uncompleted = [p for p in self.task.pois.values() if p.status != PoiStatus.COMPLETED]
        if uncompleted:
            # Only pull toward uncompleted POIs that are still PENDING (ours to claim)
            pending = [p for p in uncompleted if p.status in (PoiStatus.PENDING, PoiStatus.REASSIGN_REQUIRED, PoiStatus.DEFERRED)]
            if pending:
                nearest = min(pending, key=lambda p: dist(self.t.position, p.pos))
                return (nearest.pos - self.t.position).hnormalized()
            # All remaining POIs are being handled by other drones — hold position
            hold = self.last_completed_poi_pos or Vec3(50.0, 50.0, -self.cfg.cruise_altitude_m)
            delta = hold - self.t.position
            if delta.hnorm() > 1.0:   # only pull if meaningfully far from hold point
                return delta.hnormalized()
            return Vec3()  # already at hold point — zero direction is fine (no cruise step added below)
        # All POIs completed — hold at last completed position or field center
        hold = self.last_completed_poi_pos or Vec3(50.0, 50.0, -self.cfg.cruise_altitude_m)
        delta = hold - self.t.position
        if delta.hnorm() > 1.0:
            return delta.hnormalized()
        return Vec3()

    def log_cycle(self, od, rs, min_sep):
        c = self.task.current(); conn = sum(1 for p in self.peers.values() if p.connected and p.healthy)
        al = sum(self.latencies_ms)/len(self.latencies_ms) if self.latencies_ms else 0.0
        gr = self.gcs_direct or self.gcs_hop_count is not None
        row = {
            "timestamp": time.time(), "uav_id": self.cfg.uav_id, "mode": self.mode.value,
            "x_n": self.t.position.x, "y_e": self.t.position.y, "z_d": self.t.position.z,
            "vx": self.t.velocity.x, "vy": self.t.velocity.y, "vz": self.t.velocity.z,
            "battery_pct": self.t.battery_pct, "armed": int(self.t.armed), "flight_mode": self.t.mode,
            "poi_id": self.task.assigned or "", "poi_status": c.status.value if c else PoiStatus.PENDING.value,
            "cluster_id": self.cluster_id, "neighbor_count": len(self.peers),
            "connected_neighbors": conn, "nearest_neighbor_m": self.nearest_neighbor if math.isfinite(self.nearest_neighbor) else "",
            "next_hop": self.next_hop_id if self.next_hop_id is not None else "",
            "gcs_direct": int(self.gcs_direct), "gcs_reachable": int(gr),
            "gcs_hop_count": self.gcs_hop_count if self.gcs_hop_count is not None else "",
            "route_quality_db": self.gcs_route_quality_db, "route_pdr": self.gcs_route_pdr,
            "strongest_relay": self.last_relay if self.last_relay is not None else "",
            "relay_score": rs, "obstacle_distance_m": od, "formation_error_m": self.formation_error,
            "network_deformation_m": self.network_deformation,
            "minimum_separation_m": min_sep if math.isfinite(min_sep) else "",
            "geofence_violation": int(self.geofence_violation), "collision_violation": int(self.collision_violation),
            "link_at_risk": int(self.link_at_risk), "rth_triggered": int(self.rth_triggered),
            "recovery_event": self.recovery_event, "recovery_count": self.recovery_count,
            "recovery_time_last_s": self.recovery_time_last_s, "pdr_tx": self.tx_packets,
            "pdr_rx": self.rx_packets, "forwarded_packets": self.forwarded_packets,
            "delivered_to_gcs": self.delivered_to_gcs, "dropped_packets": self.dropped_packets,
            "latency_ms_avg": al, "connectivity_downtime_s": self.connectivity_downtime_s,
            "relay_reallocations": self.relay_changes, "bridge_active": int(self.bridge_active)
        }
        self.writer.writerow(row); self.log.flush()

    def startup(self, arm=False, takeoff=None):
        if self.dry_run:
            sx = (self.cfg.uav_id - 1) * self.cfg.min_separation_m * 2.0
            self.t = Telemetry(Vec3(sx, 0, -10), Vec3(), 100.0, False, "DRY_RUN", True)
            self.mode = DroneMode.NORMAL; self.refresh_local_connectivity(time.monotonic()); return
        if self.ap is None: raise RuntimeError("AP required")
        self.ap.heartbeat(); self.ap.request_streams()
        # ── Pre-flight consensus: decide POI before motors spin ──────────
        print(f"[UAV {self.cfg.uav_id}] Pre-flight planning (5 rounds)…")
        for _r in range(5):
            self.ap.pump(self.t)
            _now = time.monotonic()
            self.refresh_local_connectivity(_now)
            self.receive(_now)
            self.broadcast_bids(_now)
            self.elect_hierarchy()
            self.maybe_assign_task(_now)
            if self.task.assigned:
                print(f"[UAV {self.cfg.uav_id}] Pre-flight: claimed {self.task.assigned} as {self.cluster_role}/{self.group_role}")
                break
            time.sleep(0.2)
        if not self.task.assigned:
            print(f"[UAV {self.cfg.uav_id}] Pre-flight: no task yet (will retry in flight)")
        # ─────────────────────────────────────────────────────────────────
        self.ap.guided()
        if arm: self.ap.arm()
        if takeoff is not None:
            if not arm: raise RuntimeError("Use --arm with --takeoff")
            self.ap.takeoff(takeoff)
        self.mode = DroneMode.NORMAL

    def run(self, duration):
        period = 1.0 / max(self.cfg.control_hz, 0.1); start = time.monotonic()
        try:
            while True:
                ls = time.monotonic(); self.cycle()
                if self.mode == DroneMode.COMPLETE:
                    print(f"[UAV {self.cfg.uav_id}] Mission complete! Initiating RTL sequence.")
                    break
                if duration > 0 and time.monotonic() - start >= duration: break
                time.sleep(max(0.0, period - (time.monotonic() - ls)))
        except (KeyboardInterrupt, SystemExit, Exception) as e:
            print(f"\n[UAV-X] stopping UAV {self.cfg.uav_id}: {e}")
        finally:
            try:
                if isinstance(self.ap, PX4Offboard):
                    try:
                        self.ap.pump(self.t)
                        if self.t.armed:
                            print(f"[PX4] UAV {self.cfg.uav_id} requesting RTL.")
                            self.ap.rtl()
                    except: pass
            finally:
                if self.bus:
                    try: self.bus.close()
                    except: pass
                try: self.log.close()
                except: pass
                if isinstance(self.ap, PX4Offboard):
                    try: self.ap.destroy_node()
                    except: pass
                    try:
                        if rclpy is not None and rclpy.ok(): rclpy.shutdown()
                    except: pass
                print(f"[UAV-X] log: {self.log_path}")

def load_pois(path):
    if not path:
        return [
            # All POIs are at x>=55 OR y>=55 — at least 20m clear of spawn grid (15,15)-(35,35)
            Poi("POI-1",  55, 20, -5, 3),  # right side south  — HIGH priority
            Poi("POI-2",  70, 20, -5, 2),  # right side south  — MED
            Poi("POI-3",  85, 20, -5, 1),  # far-right corner  — LOW
        ]
    data = json.loads(Path(path).read_text())
    return [Poi(str(i["poi_id"]), float(i["x"]), float(i["y"]), float(i["z"]), int(i.get("priority", 1))) for i in data]

def default_geofence(): return [Vec3(0, 0, 0), Vec3(100, 0, 0), Vec3(100, 100, 0), Vec3(0, 100, 0)]

def load_sim_obstacles(world_path: str) -> List[SimObstacle]:
    world_file = Path(world_path)
    if not world_file.is_file():
        raise FileNotFoundError(f"Gazebo world file not found: {world_file}")
    root = ET.parse(world_file).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"Gazebo world file has no <world> element: {world_file}")

    def pose_values(element) -> Tuple[float, float, float, float]:
        pose = element.findtext("pose")
        values = [float(value) for value in pose.split()] if pose else []
        values += [0.0] * (6 - len(values))
        return values[0], values[1], values[2], values[5]

    obstacles = []
    for model in world.findall("model"):
        if not model.get("name", "").startswith("obstacle_"):
            continue
        mx, my, _, myaw = pose_values(model)
        for link in model.findall("link"):
            lx, ly, _, lyaw = pose_values(link)
            for collision in link.findall("collision"):
                cx, cy, _, cyaw = pose_values(collision)
                geometry = collision.find("geometry")
                if geometry is None or len(geometry) == 0:
                    raise ValueError(f"Obstacle collision has no geometry: {model.get('name')}")
                shape = geometry[0]
                x = mx + lx + cx
                y = my + ly + cy
                yaw = myaw + lyaw + cyaw
                if shape.tag == "box":
                    size = [float(value) for value in shape.findtext("size", "").split()]
                    if len(size) < 2 or min(size[:2]) <= 0.0:
                        raise ValueError(f"Invalid box size in obstacle {model.get('name')}")
                    obstacles.append(SimObstacle("box", x, y, width=size[0], depth=size[1], yaw=yaw))
                elif shape.tag == "cylinder":
                    radius = float(shape.findtext("radius", "0"))
                    if radius <= 0.0:
                        raise ValueError(f"Invalid cylinder radius in obstacle {model.get('name')}")
                    obstacles.append(SimObstacle("cylinder", x, y, radius=radius))
                elif shape.tag == "sphere":
                    radius = float(shape.findtext("radius", "0"))
                    if radius <= 0.0:
                        raise ValueError(f"Invalid sphere radius in obstacle {model.get('name')}")
                    obstacles.append(SimObstacle("sphere", x, y, radius=radius))
                else:
                    raise ValueError(
                        f"Unsupported Gazebo obstacle shape '{shape.tag}' in {model.get('name')}"
                    )
    if not obstacles:
        raise ValueError(f"No obstacle_ models with supported collision geometry found in {world_file}")
    return obstacles

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--duration", type=float, default=20.0)
    p.add_argument("--backend", choices=["px4", "ardupilot"], default="px4")
    p.add_argument("--connect", default="udpin:127.0.0.1:14550")
    p.add_argument("--namespace", default="px4_1")
    p.add_argument("--uav-id", type=int, default=1)
    p.add_argument("--swarm-size", type=int, default=5)
    p.add_argument("--origin-x", type=float, default=0.0)
    p.add_argument("--origin-y", type=float, default=0.0)
    p.add_argument("--pois", default=None)
    p.add_argument("--world", default=str(Path(__file__).resolve().parent / "Tools/simulation/gz/worlds/default.sdf"))
    p.add_argument("--log-dir", default="uav_x_logs")
    p.add_argument("--arm", action="store_true")
    p.add_argument("--takeoff", type=float, default=None)
    p.add_argument("--sim-obstacles", action="store_true", default=True)
    p.add_argument("--gcs-x", type=float, default=0.0)
    p.add_argument("--gcs-y", type=float, default=0.0)
    p.add_argument("--gcs-z", type=float, default=0.0)
    return p.parse_args()

def main():
    args = parse_args()
    if not 1 <= args.swarm_size <= 16:
        raise SystemExit("--swarm-size must be between 1 and 16.")
    if not 1 <= args.uav_id <= args.swarm_size:
        raise SystemExit("--uav-id must be between 1 and --swarm-size.")
    cfg = Config(uav_id=args.uav_id, swarm_size=args.swarm_size, log_dir=args.log_dir)
    cfg.gcs_position = Vec3(args.gcs_x, args.gcs_y, args.gcs_z)
    pois = load_pois(args.pois); fence = default_geofence()
    obs = SimStereoToFSensor(load_sim_obstacles(args.world)) if args.sim_obstacles else NullObstacleProvider()
    ap = None
    if not args.dry_run:
        if args.backend == "px4":
            origin = Vec3(args.origin_x, args.origin_y, 0.0)
            ap = PX4Offboard(args.namespace, cfg.uav_id, origin)
    node = IndividualDrone(cfg, ap, pois, fence, obs, args.dry_run)
    print("[UAV-X] v4 individual node")
    print(f"[UAV-X] UAV={cfg.uav_id}/{cfg.swarm_size}")
    print(f"[UAV-X] derived R_max={cfg.r_max_m:.2f} m, R_safe={cfg.r_safe_m:.2f} m")
    node.startup(arm=args.arm, takeoff=args.takeoff)
    node.run(args.duration)
    return 0

if __name__ == "__main__": raise SystemExit(main())
