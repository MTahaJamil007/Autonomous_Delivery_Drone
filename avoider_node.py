import math
import socket
import time
 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
 
UDP_IP   = "127.0.0.1"
UDP_PORT = 5006
 
_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
 
 
class LidarAvoider(Node):
 
    # ── tuneable constants ──────────────────────────────────────────────────
    SAFE_DIST        = 6.5   # m  – trigger avoidance
    CLEAR_DIST       = 9.0   # m  – front must exceed this to START clear timer
    CLEAR_CONFIRM_S  = 2.5   # s  – how long front must stay > CLEAR_DIST to unlock
    MIN_LOCK_S       = 2.0   # s  – minimum dodge duration before even checking clear
    FRONT_HALF_DEG   = 35    # °  – half-angle of front cone (±35° = 70° total)
    SIDE_START_DEG   = 40    # °  – side sectors start here
    SIDE_END_DEG     = 100   # °  – side sectors end here
    INF_REPLACE      = 15.0  # m  – value substituted for inf/nan readings
    # ───────────────────────────────────────────────────────────────────────
 
    def __init__(self):
        super().__init__("lidar_avoider")
 
        self._dodge_dir        = None   # None = CLEAR
        self._dodge_start_t    = 0.0
        self._clear_first_seen = 0.0
 
        self.create_subscription(LaserScan, "/lidar/scan",
                                 self._cb, 10)
 
        self.get_logger().info(
            "✅ Avoider V4 (Time-Hysteresis) Online! Broadcasting on port 5006…"
        )
 
    # ── helpers ─────────────────────────────────────────────────────────────
 
    @staticmethod
    def _median(values):
        if not values:
            return LidarAvoider.INF_REPLACE
        s = sorted(values)
        return s[len(s) // 2]
 
    def _sector(self, ranges, n, angle_min_rad, angle_inc_rad,
                start_deg, end_deg):
        """Return cleaned values in [start_deg, end_deg] (positive = CW from front)."""
        out = []
        for deg in range(start_deg, end_deg):
            idx = round((math.radians(deg) - angle_min_rad) / angle_inc_rad)
            idx = idx % n
            r = ranges[idx]
            out.append(r if (math.isfinite(r) and r > 0.05) else self.INF_REPLACE)
        return out
 
    # ── main callback ────────────────────────────────────────────────────────
 
    def _cb(self, msg: LaserScan):
        now   = time.time()
        n     = len(msg.ranges)
        amin  = msg.angle_min
        ainc  = msg.angle_increment
 
        raw = list(msg.ranges)
 
        # Front cone  (−FRONT_HALF° … +FRONT_HALF°)
        front = []
        for deg in range(-self.FRONT_HALF_DEG, self.FRONT_HALF_DEG + 1):
            idx = round((math.radians(deg) - amin) / ainc) % n
            r   = raw[idx]
            front.append(r if (math.isfinite(r) and r > 0.05) else self.INF_REPLACE)
 
        # Side sectors (left = +deg, right = -deg for a 360 CCW scan)
        left  = self._sector(raw, n, amin, ainc,
                             self.SIDE_START_DEG,  self.SIDE_END_DEG)
        right = self._sector(raw, n, amin, ainc,
                             -self.SIDE_END_DEG,  -self.SIDE_START_DEG)
 
        med_front = self._median(front)
        min_front = min(front)               # keep min for instant danger
        eff_front = min(med_front, min_front)
 
        med_left  = self._median(left)
        med_right = self._median(right)
 
        # ── state machine ────────────────────────────────────────────────────
 
        if eff_front < self.SAFE_DIST:
            # ●  OBSTACLE DETECTED
            self._clear_first_seen = 0.0          # reset clear timer
 
            if self._dodge_dir is None:
                # First time: pick direction from which side is more open
                self._dodge_dir     = "DODGE_LEFT" if med_left >= med_right else "DODGE_RIGHT"
                self._dodge_start_t = now
                self.get_logger().warn(
                    f"🚨 WALL at {eff_front:.1f} m → {self._dodge_dir}"
                )
            else:
                self.get_logger().warn(
                    f"⚠️  WALL at {eff_front:.1f} m  LOCKED→ {self._dodge_dir}"
                )
 
            action = self._dodge_dir
 
        elif self._dodge_dir is not None:
            # ●  WAS DODGING – check if we can unlock
            time_dodging = now - self._dodge_start_t
 
            if time_dodging < self.MIN_LOCK_S:
                # Too soon to even consider clearing
                action = self._dodge_dir
                self.get_logger().info(
                    f"🔒 Min-lock {time_dodging:.1f}/{self.MIN_LOCK_S} s"
                )
 
            elif eff_front >= self.CLEAR_DIST:
                # Front looks open – start/accumulate clear timer
                if self._clear_first_seen == 0.0:
                    self._clear_first_seen = now
 
                clear_dur = now - self._clear_first_seen
 
                if clear_dur >= self.CLEAR_CONFIRM_S:
                    self.get_logger().info(
                        f"✅ Path CONFIRMED clear ({clear_dur:.1f} s). Unlocking."
                    )
                    self._dodge_dir        = None
                    self._clear_first_seen = 0.0
                    action = "CLEAR"
                else:
                    action = self._dodge_dir
                    self.get_logger().info(
                        f"⏳ Confirming clear {clear_dur:.1f}/{self.CLEAR_CONFIRM_S} s"
                    )
 
            else:
                # Grey zone (SAFE < front < CLEAR): keep dodging, reset clear timer
                self._clear_first_seen = 0.0
                action = self._dodge_dir
 
        else:
            # ●  TRULY CLEAR
            action = "CLEAR"
 
        _sock.sendto(action.encode(), (UDP_IP, UDP_PORT))
 
 
def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
 
 
if __name__ == "__main__":
    main()
