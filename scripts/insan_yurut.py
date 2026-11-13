#!/usr/bin/env python3
import math, sys, select, termios, tty, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates, EntityState
from gazebo_msgs.srv import SetEntityState

def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

def normalize_angle(a):
    while a > math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a

class SifirCarpismaKalabalik(Node):
    def __init__(self):
        super().__init__('sifir_carpisma_kalabalik')
        self.init_poses = {
            'insan_1': (-8.0, -1.7, 1.57), 'insan_2': (-4.8, 1.7, -1.57),
            'insan_3': (-1.6, -1.7, 1.57), 'insan_4': (1.6, 1.7, -1.57),
            'insan_5': (4.8, -1.7, 1.57),  'insan_6': (8.0, 1.7, -1.57),
            'insan_7': (-9.5, 1.1, 0.00),  'insan_8': (9.5, -1.1, 3.14),
            'insan_9': (-6.5, -0.9, 0.00), 'insan_10': (6.5, 0.9, 3.14),
            'insan_11': (-3.0, 1.2, 0.00), 'insan_12': (3.0, -1.2, 3.14),
        }
        self.routes = {
            'insan_1': ((-8.0, -1.35), (-8.0, 1.35), 0.44), 'insan_2': ((-4.8, 1.35), (-4.8, -1.35), 0.41),
            'insan_3': ((-1.6, -1.35), (-1.6, 1.35), 0.46), 'insan_4': ((1.6, 1.35), (1.6, -1.35), 0.43),
            'insan_5': ((4.8, -1.35), (4.8, 1.35), 0.45),  'insan_6': ((8.0, 1.35), (8.0, -1.35), 0.40),
            'insan_7': ((-9.0, 0.95), (9.0, 0.95), 0.50),   'insan_8': ((9.0, -0.95), (-9.0, -0.95), 0.48),
            'insan_9': ((-6.5, -0.70), (8.5, -0.70), 0.45), 'insan_10': ((6.5, 0.70), (-8.5, 0.70), 0.47),
            'insan_11': ((-3.0, 1.10), (9.0, 1.10), 0.44),  'insan_12': ((3.0, -1.10), (-9.0, -1.10), 0.43),
        }
        self.paused = False
        self.pubs, self.targets, self.speeds = {}, {}, {}
        self.poses = {k: list(v) for k, v in self.init_poses.items()}
        self.real_poses, self.odom_offsets = {}, {}
        self.last_t = time.monotonic()
        for name, (pt_a, pt_b, speed) in self.routes.items():
            self.pubs[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            self.targets[name] = [pt_b, pt_a]
            self.speeds[name] = speed
            self.create_subscription(Odometry, f'/{name}/odom', lambda msg, n=name: self.odom_cb(n, msg), 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.states_cb, 10)
        self.create_subscription(ModelStates, '/model_states', self.states_cb, 10)
        self.cli_gz = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.cli_root = self.create_client(SetEntityState, '/set_entity_state')
        self.create_timer(0.04, self.control_loop)
        print("\n[AKTIF] 12 insan carpisma onlemeli yuruyor. (SPACE/P: Durdur-Baslat | Q: Cikis)\n")

    def odom_cb(self, name, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        ix, iy, _ = self.init_poses[name]
        if name not in self.odom_offsets:
            self.odom_offsets[name] = (ix - p.x, iy - p.y) if math.hypot(p.x, p.y) < 0.35 else (0.0, 0.0)
        ox, oy = self.odom_offsets[name]
        self.real_poses[name] = (p.x + ox, p.y + oy, yaw_from_quat(q))

    def states_cb(self, msg):
        for name, pose in zip(msg.name, msg.pose):
            if name in self.init_poses or name == 'rosbot2r':
                self.real_poses[name] = (pose.position.x, pose.position.y, yaw_from_quat(pose.orientation))

    def stop_all(self):
        for pub in self.pubs.values(): pub.publish(Twist())

    def check_keyboard(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1).lower()
            if ch in (' ', 'p'):
                self.paused = not self.paused
                if self.paused:
                    self.stop_all()
                    print("\r[DURDURULDU] Sabitlendi. (Devam: SPACE) ", end="", flush=True)
                else:
                    self.last_t = time.monotonic()
                    print("\r[DEVAM] Yuruyor...                       ", end="", flush=True)
            elif ch in ('q', '\x03'):
                raise KeyboardInterrupt

    def control_loop(self):
        self.check_keyboard()
        now = time.monotonic()
        dt = min(0.08, now - self.last_t)
        self.last_t = now
        if self.paused:
            self.stop_all(); return
        cli = self.cli_gz if self.cli_gz.service_is_ready() else (self.cli_root if self.cli_root.service_is_ready() else None)
        cur = {}
        for name in self.routes:
            if name in self.real_poses:
                cur[name] = self.real_poses[name]
                self.poses[name] = list(self.real_poses[name])
            else:
                cur[name] = tuple(self.poses[name])
        if 'rosbot2r' in self.real_poses: cur['rosbot2r'] = self.real_poses['rosbot2r']

        for name, speed in self.speeds.items():
            my_idx = int(name.split('_')[1])
            mx, my, myaw = cur[name]
            tx, ty = self.targets[name][0]
            if math.hypot(tx - mx, ty - my) < 0.42 or (my > 1.48 and ty > my) or (my < -1.48 and ty < my) or (mx > 9.4 and tx > mx) or (mx < -9.4 and tx < mx):
                self.targets[name].reverse()
                tx, ty = self.targets[name][0]
            ga = math.atan2(ty - my, tx - mx)
            vx, vy = speed * math.cos(ga), speed * math.sin(ga)
            must_yield, nearest = False, 99.0
            for oname, (ox, oy, _) in cur.items():
                if oname == name: continue
                dx, dy = mx - ox, my - oy
                dist = math.hypot(dx, dy)
                if dist <= 0.01 or dist > 1.55: continue
                if dist < nearest: nearest = dist
                in_front = abs(normalize_angle(math.atan2(-dy, -dx) - myaw)) < 0.75
                i_yield = True if oname == 'rosbot2r' else (my_idx > int(oname.split('_')[1]))
                if in_front and dist < 0.85: must_yield = True
                ux, uy = dx / dist, dy / dist
                force = ((1.55 - dist) / 1.55) ** 2 * 1.65
                vx += (ux * 0.80 + uy * 0.55) * force
                vy += (uy * 0.80 - ux * 0.55) * force
            if my > 1.42: vy -= (my - 1.42) * 5.5
            elif my < -1.42: vy += (-1.42 - my) * 5.5
            if mx > 9.1: vx -= (mx - 9.1) * 5.5
            elif mx < -9.1: vx += (-9.1 - mx) * 5.5
            yaw_err = normalize_angle(math.atan2(vy, vx) - myaw)
            msg = Twist()
            if abs(yaw_err) > 1.15:
                msg.linear.x, msg.angular.z = speed * 0.25, (2.8 if yaw_err > 0 else -2.8)
            else:
                msg.angular.z = max(-2.8, min(2.8, 3.2 * yaw_err))
                base_v = speed * max(0.55, math.cos(yaw_err))
                msg.linear.x = base_v * (0.75 if must_yield else 1.0)
            nyaw = normalize_angle(myaw + msg.angular.z * dt)
            cx = max(-9.4, min(9.4, mx + msg.linear.x * math.cos(nyaw) * dt))
            cy = max(-1.55, min(1.55, my + msg.linear.x * math.sin(nyaw) * dt))
            if any(on != name and math.hypot(cx - ox, cy - oy) < 0.56 and math.hypot(cx - ox, cy - oy) < math.hypot(mx - ox, my - oy) for on, (ox, oy, _) in cur.items()):
                msg.linear.x, cx, cy = 0.0, mx, my
            self.poses[name] = [cx, cy, nyaw]
            self.pubs[name].publish(msg)
            if cli is not None:
                req = SetEntityState.Request()
                st = EntityState(name=name)
                st.pose.position.x, st.pose.position.y, st.pose.position.z = float(cx), float(cy), 0.046
                st.pose.orientation.x, st.pose.orientation.y, st.pose.orientation.z, st.pose.orientation.w = quat_from_yaw(nyaw)
                req.state = st
                cli.call_async(req)

def main():
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    rclpy.init()
    node = SifirCarpismaKalabalik()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.stop_all()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print("\n[CIKIS] Durduruldu.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
