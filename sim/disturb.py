#!/usr/bin/python3
"""Push a torque onto the airframe in Gazebo (roll/pitch disturbance pulses).

Runs under the *system* python (/usr/bin/python3): the Gazebo transport
bindings (gz.transport13, gz.msgs10) are not installed in the project venv.

Two ways to use it:

  1. As a helper process driven by fly_plan_mission.py over stdin, one command
     per line:
         body <L> <M> <N> <roll> <pitch> <yaw>   torque L,M,N (N.m, body FRD)
                                                 at attitude roll,pitch,yaw (rad)
         clear                                   remove the torque
     It answers "ok" per line.

  2. By hand, for a quick check:
         /usr/bin/python3 sim/disturb.py size 6      # print moment for 6 m/s
         /usr/bin/python3 sim/disturb.py test roll 1.4 1.0

Frames. The world is ENU. ApplyLinkWrench applies the wrench in world axes at
the link origin, so the body torque is rotated body(FRD) -> NED -> ENU using
the attitude at the start of the pulse and held fixed for the pulse.

Size. A gust of speed u_g on one semi-span gives roughly
    M = 1/2 rho u_g^2 * S * b/4
with S = 0.2044 m^2 and b = 1.30 m (OpenVSP reference values of this wing).
That is 1.44 N.m at 6 m/s and 3.24 N.m at 9 m/s.
"""
import math
import sys
import time

WORLD = "zephyr_runway"
LINK = "My_QuadPlane::base_link"
RHO = 1.2041
S_WING = 0.2044
SPAN = 1.30


def gust_moment(u_g):
    return 0.5 * RHO * u_g * u_g * S_WING * SPAN / 4.0


def body_to_enu(L, M, N, roll, pitch, yaw):
    """Rotate a body-FRD vector to world ENU (ZYX Euler, NED)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # R_nb (body -> NED)
    n = (cp * cy) * L + (sr * sp * cy - cr * sy) * M + (cr * sp * cy + sr * sy) * N
    e = (cp * sy) * L + (sr * sp * sy + cr * cy) * M + (cr * sp * sy - sr * cy) * N
    d = (-sp) * L + (sr * cp) * M + (cr * cp) * N
    return e, n, -d


class Wrench:
    def __init__(self):
        from gz.transport13 import Node
        from gz.msgs10.entity_wrench_pb2 import EntityWrench
        from gz.msgs10.entity_pb2 import Entity
        self.EntityWrench, self.Entity = EntityWrench, Entity
        self.node = Node()
        self.pub = self.node.advertise("/world/%s/wrench/persistent" % WORLD, EntityWrench)
        self.clr = self.node.advertise("/world/%s/wrench/clear" % WORLD, Entity)
        # A message published before discovery has connected Gazebo's wrench
        # subscriber is silently dropped (publish() still returns True). A fixed
        # 1 s wait was not enough when the simulation runs faster than real
        # time: the first pulse of every run was lost. Wait for the connection.
        t0 = time.time()
        while not (self.pub.has_connections() and self.clr.has_connections()):
            if time.time() - t0 > 30.0:
                raise SystemExit("disturb.py: Gazebo wrench topics never connected")
            time.sleep(0.05)
        time.sleep(0.2)

    def apply(self, tx, ty, tz):
        msg = self.EntityWrench()
        msg.entity.name = LINK
        msg.entity.type = self.Entity.LINK
        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = tx, ty, tz
        return self.pub.publish(msg)

    def clear(self):
        msg = self.Entity()
        msg.name = LINK
        msg.type = self.Entity.LINK
        return self.clr.publish(msg)


def serve():
    w = Wrench()
    print("ready", flush=True)
    for line in sys.stdin:
        f = line.split()
        if not f:
            continue
        if f[0] == "body" and len(f) == 7:
            v = [float(x) for x in f[1:]]
            ok = w.apply(*body_to_enu(*v))
        elif f[0] == "clear":
            ok = w.clear()
        else:
            ok = False
        print("ok" if ok else "err", flush=True)
    w.clear()


def main():
    if len(sys.argv) >= 2 and sys.argv[1] == "size":
        for u in [float(x) for x in sys.argv[2:]] or [6.0, 9.0]:
            print("u_g %.1f m/s -> M = %.2f N.m" % (u, gust_moment(u)))
    elif len(sys.argv) == 5 and sys.argv[1] == "test":
        axis, nm, secs = sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
        L, M = (nm, 0.0) if axis == "roll" else (0.0, nm)
        w = Wrench()
        w.apply(*body_to_enu(L, M, 0.0, 0.0, 0.0, 0.0))
        time.sleep(secs)
        w.clear()
    else:
        serve()


if __name__ == "__main__":
    main()
