#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import Fall
from std_msgs.msg import Float32MultiArray
from smbus2 import SMBus
import time
import math
import numpy as np


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # ---------------------------
        # I2C / IMU initialization
        # ---------------------------
        self.bus = None
        self.bus_ready = False
        self.pending_bus_reset = False
        self.last_bus_retry_t = None
        self.bus_retry_interval_ms = 500.0
        self.init_bus(1, blocking=True)  # I2C bus 1
        self.addr_imu = 0x69             # MPU-9250 address (0x68 or 0x69)

        # Sampling interval
        self.dt = 0.02  # 50 Hz

        # Complementary filter weight (for roll/pitch monitoring only)
        self.alpha = 0.98

        # ---- Parameters (sequential fall detection) ----
        self.declare_parameter('tilt_threshold_deg', 70.0)   # posture angle threshold
        self.declare_parameter('fast_rate_thr_dps', 120.0)   # fast rotation while tilting
        self.declare_parameter('impact_thr_g', 2.2)          # impact after tilt
        self.declare_parameter('impact_timeout_ms', 1500)    # time window to wait for impact after fast tilt
        self.declare_parameter('tilt_cross_dwell_ms', 30)     # require tilt above threshold for this long
        self.declare_parameter('gyro_comp_alpha', 0.6)      # complementary filter weight for gyro prediction (0~1, high=slower)
        self.declare_parameter('arm_dwell_ms', 0)            # require being below tilt for this long to arm

        # Read parameters
        tilt_thr_deg  = float(self.get_parameter('tilt_threshold_deg').value)
        self.tilt_threshold = math.radians(tilt_thr_deg)
        self.fast_rate_thr_dps = float(self.get_parameter('fast_rate_thr_dps').value)
        self.impact_thr_g     = float(self.get_parameter('impact_thr_g').value)
        self.impact_timeout_ms = int(self.get_parameter('impact_timeout_ms').value)
        self.tilt_cross_dwell_ms = int(self.get_parameter('tilt_cross_dwell_ms').value)
        self.gyro_comp_alpha = float(self.get_parameter('gyro_comp_alpha').value)
        self.arm_dwell_ms = int(self.get_parameter('arm_dwell_ms').value)

        # Publishers
        self.pub_rp    = self.create_publisher(Float32MultiArray, '/imu/roll_pitch', 10)
        self.pub_tilt  = self.create_publisher(Float32MultiArray, '/imu/tilt', 10)
        self.pub_alert = self.create_publisher(Fall, '/imu/alert', 10)

        # Internal states (roll/pitch only for display)
        self.roll = 0.0
        self.pitch = 0.0

        # ---- State machine (tilt -> fast tilt -> impact) ----
        self.state = "IDLE"
        self.t_state = self.get_clock().now()
        self.tilt_detected_t = None
        self.fast_tilt_t = None
        self.last_tilt_rad = 0.0
        self.last_impact_t = None
        self.initial_pose = {}
        self.prev_tilt_over = False
        self.last_under_tilt_t = self.get_clock().now()
        self.initial_g_hat = None
        self.initial_ref_x = None
        self.initial_ref_y = None
        self.up_est = None
        self.tilt_over_start_t = None

        # IMU setup (calibration assumes 수직 자세라, 기울어진 상태에서 시작할 땐 False 권장)
        self._setup_imu(calibrate=False)
        self._capture_initial_pose()

        # Timer
        self.create_timer(self.dt, self.timer_cb)

        self.get_logger().info(
            f'FallDetectionNode started: tilt_thr={tilt_thr_deg:.1f} deg, '
            f'fast_rate={self.fast_rate_thr_dps:.0f} dps, impact_thr={self.impact_thr_g:.1f} g, '
            f'impact_timeout={self.impact_timeout_ms} ms'
        )

    # ---------------------------
    # Low-level I2C / registers
    # ---------------------------
    def init_bus(self, bus_num: int, blocking=True):
        try:
            if self.bus is not None:
                self.bus.close()
        except Exception:
            pass

        def attempt():
            self.bus = SMBus(bus_num)
            self.bus_ready = True
            self.pending_bus_reset = False
            self.last_bus_retry_t = None
            self.get_logger().info('I2C-1 initialized successfully')

        if blocking:
            while True:
                try:
                    attempt()
                    break
                except OSError as e:
                    self.bus_ready = False
                    self.get_logger().warn(f'I2C initialization failed: {e!r}, retrying in 5 sec...')
                    time.sleep(5)
        else:
            try:
                attempt()
                return True
            except OSError as e:
                self.bus_ready = False
                self.get_logger().warn(f'I2C initialization failed: {e!r}; will retry')
                return False

    def _wakeup_imu(self):
        # PWR_MGMT_1(0x6B) = 0x00
        self.bus.write_byte_data(self.addr_imu, 0x6B, 0x00)
        time.sleep(0.1)

    def _config_imu(self):
        # GYRO_CONFIG(0x1B): +-250 dps -> 0x00
        # ACCEL_CONFIG(0x1C): +-2 g    -> 0x00
        self.bus.write_byte_data(self.addr_imu, 0x1B, 0x00)
        self.bus.write_byte_data(self.addr_imu, 0x1C, 0x00)
        # Faster bandwidths (optional)
        self.bus.write_byte_data(self.addr_imu, 0x1A, 0x01)  # ~188 Hz gyro
        self.bus.write_byte_data(self.addr_imu, 0x1D, 0x01)  # ~184 Hz accel
        time.sleep(0.05)

    def _bypass_enable(self):
        try:
            self.bus.write_byte_data(self.addr_imu, 0x37, 0x02)
            time.sleep(0.05)
        except Exception:
            pass

    def _setup_imu(self, calibrate=False):
        self._wakeup_imu()
        self._config_imu()
        self._bypass_enable()
        self._load_scales()
        if calibrate:
            self._quick_calibration(1.0)

    def _read_word(self, addr, reg):
        hi = self.bus.read_byte_data(addr, reg)
        lo = self.bus.read_byte_data(addr, reg + 1)
        val = (hi << 8) | lo
        return val - 65536 if (val & 0x8000) else val
    
    # ---------------------------
    # Scale factors & bias
    # ---------------------------
    def _load_scales(self):
        ac = self.bus.read_byte_data(self.addr_imu, 0x1C)
        afs_sel = (ac >> 3) & 0x03
        acc_lut = [16384.0, 8192.0, 4096.0, 2048.0]   # LSB/g
        self.acc_scale = acc_lut[afs_sel]

        gc = self.bus.read_byte_data(self.addr_imu, 0x1B)
        fs_sel = (gc >> 3) & 0x03
        gyr_lut = [131.0, 65.5, 32.8, 16.4]           # LSB/(deg/s)
        self.gyr_scale = gyr_lut[fs_sel]

        self.get_logger().info(
            f'ACCEL_CONFIG=0x{ac:02X}(AFS_SEL={afs_sel}) -> {self.acc_scale:.1f} LSB/g, '
            f'GYRO_CONFIG=0x{gc:02X}(FS_SEL={fs_sel}) -> {self.gyr_scale:.1f} LSB/deg/s'
        )

    def _quick_calibration(self, duration_s=1.0):
        n = max(1, int(duration_s / self.dt))
        accs = []
        gyrs = []
        for _ in range(n):
            ax, ay, az = self._read_accel_raw()
            gx, gy, gz = self._read_gyro_raw()
            accs.append([ax, ay, az])
            gyrs.append([gx, gy, gz])
            time.sleep(self.dt)
        acc_mean = np.array(accs).mean(axis=0)
        gyr_mean = np.array(gyrs).mean(axis=0)

        # Assume board Z+ upward
        self.acc_bias = acc_mean - np.array([0.0, 0.0, 1.0])
        self.gyr_bias = gyr_mean

        self.get_logger().info(f'Accel bias={self.acc_bias}, Gyro bias={self.gyr_bias}')

        # Init roll/pitch (monitor only)
        ax, ay, az = acc_mean - self.acc_bias
        self.roll  = math.atan2(ay, az)
        self.pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))

    # ---------------------------
    # Sensor reading
    # ---------------------------
    def _read_accel_raw(self):
        ax = self._read_word(self.addr_imu, 0x3B) / self.acc_scale
        ay = self._read_word(self.addr_imu, 0x3D) / self.acc_scale
        az = self._read_word(self.addr_imu, 0x3F) / self.acc_scale
        return ax, ay, az

    def _read_gyro_raw(self):
        gx = self._read_word(self.addr_imu, 0x43) / self.gyr_scale
        gy = self._read_word(self.addr_imu, 0x45) / self.gyr_scale
        gz = self._read_word(self.addr_imu, 0x47) / self.gyr_scale
        return gx, gy, gz

    def read_accel(self):
        ax, ay, az = self._read_accel_raw()
        if hasattr(self, "acc_bias"):
            ax -= self.acc_bias[0]
            ay -= self.acc_bias[1]
            az -= self.acc_bias[2]
        return np.array([ax, ay, az])

    def read_gyro(self):
        gx, gy, gz = self._read_gyro_raw()
        if hasattr(self, "gyr_bias"):
            gx -= self.gyr_bias[0]
            gy -= self.gyr_bias[1]
            gz -= self.gyr_bias[2]
        return np.array([gx, gy, gz])
    
    # ---------------------------
    # Time utils
    # ---------------------------
    def _elapsed_ms(self):
        return (self.get_clock().now() - self.t_state).nanoseconds / 1e6

    def _ms_since(self, ts):
        if ts is None:
            return 1e9
        return (self.get_clock().now() - ts).nanoseconds / 1e6

    def _capture_initial_pose(self):
        # Use current acceleration vector to establish initial roll/pitch/tilt
        accel = self.read_accel()
        roll_acc  = math.atan2(accel[1], accel[2])
        pitch_acc = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))
        a_norm = max(1e-9, float(np.linalg.norm(accel)))
        tilt_rad = math.acos(max(-1.0, min(1.0, accel[2] / a_norm)))
        g_hat_init = accel / a_norm

        # Build an orthonormal basis tied to the initial "up" direction
        ref_x = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(ref_x, g_hat_init)) > 0.9:
            ref_x = np.array([0.0, 1.0, 0.0])
        ref_y = np.cross(g_hat_init, ref_x)
        norm_y = np.linalg.norm(ref_y)
        if norm_y < 1e-9:
            ref_y = np.array([0.0, 0.0, 1.0])
            norm_y = np.linalg.norm(ref_y)
        ref_y = ref_y / norm_y
        ref_x = np.cross(ref_y, g_hat_init)
        ref_x = ref_x / max(1e-9, np.linalg.norm(ref_x))

        self.roll = roll_acc
        self.pitch = pitch_acc
        self.last_tilt_rad = tilt_rad
        self.prev_tilt_over = tilt_rad >= self.tilt_threshold
        self.last_under_tilt_t = None if self.prev_tilt_over else self.get_clock().now()
        self.initial_g_hat = g_hat_init
        self.initial_ref_x = ref_x
        self.initial_ref_y = ref_y
        self.up_est = g_hat_init
        self.initial_pose = {
            "roll_deg": math.degrees(roll_acc),
            "pitch_deg": math.degrees(pitch_acc),
            "tilt_deg": math.degrees(tilt_rad),
        }
        self.get_logger().info(
            f'Initial pose set to roll={self.initial_pose["roll_deg"]:.1f} deg, '
            f'pitch={self.initial_pose["pitch_deg"]:.1f} deg, '
            f'tilt={self.initial_pose["tilt_deg"]:.1f} deg'
        )

    def _reset_to_idle(self):
        self.state = "IDLE"
        self.t_state = self.get_clock().now()
        self.tilt_detected_t = None
        self.fast_tilt_t = None
        self.prev_tilt_over = False
        self.tilt_over_start_t = None
        self.last_under_tilt_t = self.get_clock().now()

    # ---------------------------
    # Main loop
    # ---------------------------
    def timer_cb(self):
        try:
            if not self.bus_ready:
                if not self.pending_bus_reset:
                    return
                if self._ms_since(self.last_bus_retry_t) < self.bus_retry_interval_ms:
                    return
                if self.init_bus(1, blocking=False):
                    self._setup_imu(calibrate=False)
                else:
                    self.last_bus_retry_t = self.get_clock().now()
                    return
            accel = self.read_accel()     # used for tilt only
            gyro  = self.read_gyro()      # deg/s
            omega_rad = np.radians(gyro)  # rad/s

            # Roll/pitch (monitor only)
            roll_acc  = math.atan2(accel[1], accel[2])
            pitch_acc = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))
            roll_gyro  = self.roll  + math.radians(gyro[0]) * self.dt
            pitch_gyro = self.pitch + math.radians(gyro[1]) * self.dt
            self.roll  = self.alpha * roll_gyro  + (1.0 - self.alpha) * roll_acc
            self.pitch = self.alpha * pitch_gyro + (1.0 - self.alpha) * pitch_acc

            # Gravity-based tilt relative to initial "up" vector
            a_norm = float(np.linalg.norm(accel))
            if a_norm < 1e-9:
                a_norm = 1e-9
            g_hat_raw = accel / a_norm
            if self.initial_g_hat is None:
                self.initial_g_hat = g_hat_raw
            if self.up_est is None:
                self.up_est = g_hat_raw
            # Gyro prediction (small-angle)
            up_pred = self.up_est + np.cross(omega_rad, self.up_est) * self.dt
            up_pred = up_pred / max(1e-9, float(np.linalg.norm(up_pred)))
            # Complementary blend with accel
            self.up_est = self.gyro_comp_alpha * up_pred + (1.0 - self.gyro_comp_alpha) * g_hat_raw
            self.up_est = self.up_est / max(1e-9, float(np.linalg.norm(self.up_est)))
            g_hat = self.up_est

            dot_up = max(-1.0, min(1.0, float(np.dot(g_hat, self.initial_g_hat))))
            tilt_rad = math.acos(dot_up)
            tilt_deg = math.degrees(tilt_rad)
            self.last_tilt_rad = tilt_rad

            # Tilt direction relative to initial frame
            ref_x = self.initial_ref_x
            ref_y = self.initial_ref_y
            if ref_x is None or ref_y is None:
                # Fallback to world XY
                ref_x = np.array([1.0, 0.0, 0.0])
                ref_y = np.array([0.0, 1.0, 0.0])
            proj = g_hat - np.dot(g_hat, self.initial_g_hat) * self.initial_g_hat
            dir_x = float(np.dot(proj, ref_x))
            dir_y = float(np.dot(proj, ref_y))
            tilt_dir_rad = math.atan2(dir_y, dir_x) if (abs(dir_x) + abs(dir_y)) > 1e-9 else 0.0
            tilt_dir_deg = math.degrees(tilt_dir_rad)
            tilt_x_deg = tilt_deg * math.cos(tilt_dir_rad)
            tilt_y_deg = tilt_deg * math.sin(tilt_dir_rad)

            # Angular-rate features
            omega_max = max(abs(gyro[0]), abs(gyro[1]), abs(gyro[2]))  # dps
            tilt_over_raw = (tilt_rad >= self.tilt_threshold)
            if tilt_over_raw:
                if self.tilt_over_start_t is None:
                    self.tilt_over_start_t = self.get_clock().now()
            else:
                self.tilt_over_start_t = None
            dwell_met = (self.tilt_over_start_t is not None) and (self._ms_since(self.tilt_over_start_t) >= self.tilt_cross_dwell_ms)
            tilt_over = tilt_over_raw and dwell_met
            impact_now = a_norm >= self.impact_thr_g
            if not tilt_over_raw:
                self.last_under_tilt_t = self.get_clock().now()
            armed = (self.last_under_tilt_t is not None) and (self._ms_since(self.last_under_tilt_t) >= self.arm_dwell_ms)
            tilt_cross_now = tilt_over and (not self.prev_tilt_over) and armed

            # Debug
            self.get_logger().debug(
                f'STATE={self.state} | omega_max={omega_max:.1f} dps, tilt={tilt_deg:.1f} deg, |a|={a_norm:.2f} g'
            )

            # ---------- State machine ----------
            if self.state == "IDLE":
                if tilt_cross_now:
                    self.state = "TILTED"
                    self.tilt_detected_t = self.get_clock().now()
                    self.t_state = self.tilt_detected_t

            elif self.state == "TILTED":
                if not tilt_over and self._elapsed_ms() > 500:
                    self._reset_to_idle()
                elif tilt_over and omega_max >= self.fast_rate_thr_dps:
                    self.state = "WAIT_IMPACT"
                    self.fast_tilt_t = self.get_clock().now()
                    self.t_state = self.fast_tilt_t

            elif self.state == "WAIT_IMPACT":
                if impact_now and tilt_over and (self._ms_since(self.fast_tilt_t) <= self.impact_timeout_ms):
                    msg = Fall(); msg.is_fall = True
                    self.pub_alert.publish(msg)
                    self.last_impact_t = self.get_clock().now()
                    self.get_logger().warn(
                        f'FALL detected: tilt>={math.degrees(self.tilt_threshold):.0f} deg, '
                        f'fast tilt={omega_max:.0f} dps, impact={a_norm:.2f} g'
                    )
                    self._reset_to_idle()
                elif (not tilt_over) or (self._ms_since(self.fast_tilt_t) > self.impact_timeout_ms):
                    self._reset_to_idle()

             # ---------------- Publish monitors ----------------
            rp = Float32MultiArray()
            rp.data = [float(math.degrees(self.roll)), float(math.degrees(self.pitch)), float(a_norm)]
            self.pub_rp.publish(rp)

            tilt_msg = Float32MultiArray()
            tilt_msg.data = [tilt_deg, tilt_dir_deg, tilt_x_deg, tilt_y_deg]
            self.pub_tilt.publish(tilt_msg)
            self.prev_tilt_over = tilt_over

        except Exception as e:
            if self.bus_ready:
                self.get_logger().warn(f'I2C exception: {e!r} -> scheduling bus reset')
            try:
                if self.bus is not None:
                    self.bus.close()
            except Exception:
                pass
            self.bus_ready = False
            self.pending_bus_reset = True
            self.last_bus_retry_t = None

# ---------------------------
# ROS2 entrypoint
# ---------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
