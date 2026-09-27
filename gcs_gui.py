#!/usr/bin/env python3
import math
import os
import socket
import json

if not os.environ.get("DISPLAY") and not os.environ.get("MPLBACKEND"):
    import matplotlib
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


class GCS3DIso:
    def __init__(self, listen_port=15999):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", listen_port))
        self.sock.setblocking(False)
        self.swarm_data = {}
        self.running = True

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_title("PUSHPAK UAV-X Swarm Network | Isometric")
        self.ax.set_axis_off()

    @staticmethod
    def project(x, y, z):
        z_up = -z
        angle = math.radians(30)
        return (x - y) * math.cos(angle), (x + y) * math.sin(angle) + z_up

    def run(self):
        print("[GCS 3D ISO] Listening on UDP port 15999...")
        while self.running:
            try:
                raw, _ = self.sock.recvfrom(65535)
                msg = json.loads(raw.decode("utf-8"))
                if msg.get("type") == "GCS_DELIVERY":
                    payload = msg.get("payload", {})
                    uid = payload.get("uav_id", msg.get("origin_uav", -1))
                    if uid > 0:
                        self.swarm_data[uid] = payload
            except BlockingIOError:
                pass
            except (UnicodeDecodeError, json.JSONDecodeError, OSError):
                pass

            self.update_plot()
            plt.pause(0.1)

    def update_plot(self):
        self.ax.clear()
        self.ax.set_title("PUSHPAK UAV-X Swarm Network | Isometric")
        self.ax.set_axis_off()
        plot_x = []
        plot_y = []

        def draw_line(x_values, y_values, *args, **kwargs):
            self.ax.plot(x_values, y_values, *args, **kwargs)
            plot_x.extend(np.asarray(x_values).ravel().tolist())
            plot_y.extend(np.asarray(y_values).ravel().tolist())

        # Real obstacles — matches default.sdf exactly
        # Format: (x, y, shape, param1, param2, yaw_deg)
        #   cylinder: param1=radius, param2=height
        #   sphere:   param1=radius, param2=height(=2*r)
        #   box:      param1=half_width_x, param2=half_width_y
        SDF_OBSTACLES = [
            (90,  10,  "cylinder", 6,  12, 0),
            (105, 38,  "box",      7,  5,  0),
            (110, 75,  "sphere",   7,  14, 0),
            (100, 112, "box",      8,  4,  20),   # yaw=0.35 rad≈20°
            (58,  112, "cylinder", 5,  12, 0),
            (30,  105, "box",      5,  9,  0),
            (75,  70,  "sphere",   5,  10, 0),
            (130, 82,  "cylinder", 6,  12, 0),
        ]
        theta = np.linspace(0, 2 * math.pi, 37)
        for ox, oy, shape, r1, r2, yaw_deg in SDF_OBSTACLES:
            height = r2
            if shape in ("cylinder", "sphere"):
                circle_x = ox + r1 * np.cos(theta)
                circle_y = oy + r1 * np.sin(theta)
            else:  # box: r1=half-x, r2=half-y
                yaw = math.radians(yaw_deg)
                corners = np.array([
                    [ r1,  r2], [-r1,  r2], [-r1, -r2], [ r1, -r2], [ r1,  r2]
                ])
                cos_y, sin_y = math.cos(yaw), math.sin(yaw)
                rot = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
                rotated = (rot @ corners.T).T
                circle_x = ox + rotated[:, 0]
                circle_y = oy + rotated[:, 1]
            base_x, base_y = self.project(circle_x, circle_y, 0)
            top_x, top_y = self.project(circle_x, circle_y, -height)
            self.ax.fill(base_x, base_y, color="red", alpha=0.22)
            self.ax.fill(top_x, top_y, color="red", alpha=0.12)
            draw_line(base_x, base_y, color="firebrick", alpha=0.75, linewidth=1.5)
            draw_line(top_x, top_y, color="firebrick", alpha=0.55)
            n_pts = len(circle_x)
            for index in range(0, n_pts - 1, max(1, n_pts // 12)):
                self.ax.fill(
                    [base_x[index], base_x[index + 1], top_x[index + 1], top_x[index]],
                    [base_y[index], base_y[index + 1], top_y[index + 1], top_y[index]],
                    color="red", alpha=0.10,
                )
                draw_line(
                    [base_x[index], top_x[index]],
                    [base_y[index], top_y[index]],
                    color="firebrick", alpha=0.35,
                )


        gcs_x, gcs_y = self.project(0, 0, 0)
        self.ax.scatter([gcs_x], [gcs_y], c="black", s=100, marker="^", label="GCS")
        plot_x.append(gcs_x)
        plot_y.append(gcs_y)

        edges = set()
        for uid, data in self.swarm_data.items():
            position = data.get("position", {})
            x, y, z = position.get("x", 0), position.get("y", 0), position.get("z", 0)
            drone_x, drone_y = self.project(x, y, z)
            self.ax.scatter(drone_x, drone_y, c="blue", s=50, marker="o")
            self.ax.text(drone_x + 2, drone_y + 2, f"UAV {uid}", color="black", size=10, weight="bold")
            plot_x.append(drone_x)
            plot_y.append(drone_y)

            polygon = data.get("polygon_3d", [])
            if polygon:
                projected = [self.project(point[0], point[1], point[2]) for point in polygon]
                polygon_x = [point[0] for point in projected] + [projected[0][0]]
                polygon_y = [point[1] for point in projected] + [projected[0][1]]
                draw_line(polygon_x, polygon_y, "g--", alpha=0.6)

            for neighbor_id in data.get("neighbor_ids", []):
                edge = tuple(sorted((uid, neighbor_id)))
                if edge in edges or neighbor_id not in self.swarm_data:
                    continue
                edges.add(edge)
                neighbor_position = self.swarm_data[neighbor_id].get("position", {})
                neighbor_x, neighbor_y = self.project(
                    neighbor_position.get("x", 0),
                    neighbor_position.get("y", 0),
                    neighbor_position.get("z", 0),
                )
                draw_line([drone_x, neighbor_x], [drone_y, neighbor_y], "b-", alpha=0.6)

            if data.get("signal_strength_db", -999) > -80:
                draw_line([drone_x, gcs_x], [drone_y, gcs_y], "m-", alpha=0.5)

        self.ax.legend(loc="upper right")
        if plot_x and plot_y:
            padding = 12
            self.ax.set_xlim(min(plot_x) - padding, max(plot_x) + padding)
            self.ax.set_ylim(min(plot_y) - padding, max(plot_y) + padding)
        self.ax.set_aspect("equal", adjustable="box")


if __name__ == "__main__":
    GCS3DIso().run()
