#!/usr/bin/env python3
import socket, json, time, threading, sys

class GCSMonitor:
    def __init__(self, listen_port=15999, swarm_size=5, peer_base_port=16000):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", listen_port))
        self.sock.setblocking(False)
        self.swarm_data = {}
        self.swarm_size = swarm_size
        self.peer_base_port = peer_base_port
        self.running = True

    def send_override(self, uav_id, poi_id):
        """Sends a GCS_OVERRIDE command to a specific drone to break deadlocks."""
        msg = {"type": "GCS_OVERRIDE", "uav_id": uav_id, "poi_id": poi_id, "sent_ts": time.time()}
        try:
            self.sock.sendto(json.dumps(msg).encode('utf-8'), ("127.0.0.1", self.peer_base_port + uav_id - 1))
            print(f"\n[GCS] Override sent: UAV {uav_id} -> {poi_id}")
        except Exception as e:
            print(f"\n[GCS] Override failed: {e}")

    def run(self):
        print(f"[GCS] Listening on UDP port {self.sock.getsockname()[1]}...")
        print("[GCS] Press 'o' + Enter to force UAV 1 to take POI-1 (Simulate GCS Interrupt).")
        
        input_thread = threading.Thread(target=self.input_loop, daemon=True)
        input_thread.start()

        try:
            while self.running:
                try:
                    raw, _ = self.sock.recvfrom(65535)
                    msg = json.loads(raw.decode('utf-8'))
                    if msg.get("type") == "GCS_DELIVERY":
                        payload = msg.get("payload", {})
                        uav_id = payload.get("uav_id", msg.get("origin_uav", -1))
                        if uav_id > 0: self.swarm_data[uav_id] = payload
                except BlockingIOError:
                    pass
                except Exception:
                    pass
                
                self.print_dashboard()
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.running = False
            print("\n[GCS] Shutting down.")

    def input_loop(self):
        while self.running:
            try:
                cmd = input()
                if cmd.strip().lower() == 'o':
                    self.send_override(1, "POI-1")
            except EOFError:
                break

    def print_dashboard(self):
        print("\033[2J\033[H", end="") # Clear terminal
        print("="*70)
        print(" PUSHPAK UAV-X FOREGROUND GCS - LIVE SWARM DASHBOARD")
        print("="*70)
        print(f"{'UAV':<5} | {'Mode':<15} | {'PoI':<8} | {'Sig(dBm)':<10} | {'λ2':<6} | {'Obs(m)':<8}")
        print("-"*70)
        
        for uav_id in sorted(self.swarm_data.keys()):
            d = self.swarm_data[uav_id]
            print(f"{uav_id:<5} | {d.get('mode','?'):<15} | {d.get('poi_id','?'):<8} | "
                  f"{d.get('signal_strength_db', 0):<10.2f} | "
                  f"{d.get('lambda2_connectivity', 0):<6.2f} | "
                  f"{d.get('obstacle_distance', 0):<8.2f}")
        
        print("-"*70)
        print(" Obstacles: (45,15) r=8m | (70,60) r=10m")
        print(" [GCS] Press 'o' + Enter to trigger supervisory override.")

if __name__ == "__main__":
    GCSMonitor().run()
