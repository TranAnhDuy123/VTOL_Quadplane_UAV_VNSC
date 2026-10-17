#!/usr/bin/python3
"""Record a Gazebo camera, the aircraft pose and the torque events.

Runs under the system python (Gazebo transport bindings). Until SIGTERM/SIGINT:
  frames/<sim_ms>.jpg   every image on the camera topic, named by its sim time
  pose.csv              sim_s, x, y, z (ENU, m), qw, qx, qy, qz  (odometry)
  events.csv            sim_s, event (torque_on L M N / torque_off)

Torque events are taken from the same wrench topics sim/disturb.py publishes
on, stamped with the latest odometry sim time, so the video can show exactly
when the torque acts.

    /usr/bin/python3 sim/record_camera.py <out_dir> [camera_topic] [jpeg_quality]
"""
import os
import signal
import sys
import threading

from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.image_pb2 import Image
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node
from PIL import Image as PILImage

WORLD = "zephyr_runway"
MODEL = "My_QuadPlane"


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9


def main():
    out = sys.argv[1]
    topic = sys.argv[2] if len(sys.argv) > 2 else "/hover_cam"
    quality = int(sys.argv[3]) if len(sys.argv) > 3 else 90
    os.makedirs(os.path.join(out, "frames"), exist_ok=True)
    lock = threading.Lock()
    state = {"t": 0.0, "frames": 0}
    pose_f = open(os.path.join(out, "pose.csv"), "w")
    pose_f.write("sim_s,x,y,z,qw,qx,qy,qz\n")
    ev_f = open(os.path.join(out, "events.csv"), "w")
    ev_f.write("sim_s,event\n")

    def on_image(msg):
        t = stamp(msg)
        if msg.width == 0 or not msg.data:
            return
        img = PILImage.frombytes("RGB", (msg.width, msg.height), msg.data)
        img.save(os.path.join(out, "frames", "%09d.jpg" % int(round(t * 1000))), quality=quality)
        with lock:
            state["frames"] += 1

    def on_odom(msg):
        t = stamp(msg)
        p, q = msg.pose.position, msg.pose.orientation
        with lock:
            state["t"] = t
            pose_f.write("%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f\n" % (t, p.x, p.y, p.z, q.w, q.x, q.y, q.z))

    def on_wrench(msg):
        tq = msg.wrench.torque
        with lock:
            ev_f.write("%.4f,torque_on %.3f %.3f %.3f\n" % (state["t"], tq.x, tq.y, tq.z))
            ev_f.flush()

    def on_clear(msg):
        with lock:
            ev_f.write("%.4f,torque_off\n" % state["t"])
            ev_f.flush()

    node = Node()
    ok = [node.subscribe(Image, topic, on_image),
          node.subscribe(Odometry, "/model/%s/odometry" % MODEL, on_odom),
          node.subscribe(EntityWrench, "/world/%s/wrench/persistent" % WORLD, on_wrench),
          node.subscribe(Entity, "/world/%s/wrench/clear" % WORLD, on_clear)]
    if not all(ok):
        raise SystemExit("record_camera: subscribe failed %s" % ok)
    print("recording %s -> %s" % (topic, out), flush=True)

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *a: stop.set())
    signal.signal(signal.SIGINT, lambda *a: stop.set())
    while not stop.is_set():
        stop.wait(5.0)
        with lock:
            print("  sim %.1f s, %d frames" % (state["t"], state["frames"]), flush=True)
    with lock:
        pose_f.close()
        ev_f.close()
    print("stopped, %d frames" % state["frames"], flush=True)
    # skip interpreter teardown: the gz-transport bindings can abort there
    # (pybind11 GIL check), which would turn a good recording into a failure
    os._exit(0)


if __name__ == "__main__":
    main()
