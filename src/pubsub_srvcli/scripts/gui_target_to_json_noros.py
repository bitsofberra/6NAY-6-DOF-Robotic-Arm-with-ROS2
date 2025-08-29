#!/usr/bin/env python3
import os, json, time, subprocess, tkinter as tk
from tkinter import messagebox

JSON_DEFAULT = "/home/revengeofthesob/ros3_ws/src/pubsub_srvcli/src/veri.json"
SERVER_NODE  = os.environ.get("PANDA_SERVER_NODE", "/panda_move_server")  # gerekirse değiştir

def call_ros2(cmd, timeout=30):
    # subprocess ile ros2 çağır; çıktı+timeout dön
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, timeout=timeout)
    return res.returncode == 0, res.stdout

def wait_service(name: str, timeout=30):
    t0 = time.time()
    while time.time() - t0 < timeout:
        ok, out = call_ros2(["ros2", "service", "list"], timeout=5)
        if ok and any(line.strip() == name for line in out.splitlines()):
            return True
        time.sleep(0.5)
    return False

def ensure_json_array(path: str):
    if not os.path.exists(path):
        with open(path, "w") as f: json.dump([], f, indent=2)
        return
    try:
        with open(path, "r") as f: data = json.load(f)
        if not isinstance(data, list): raise ValueError
    except Exception:
        with open(path, "w") as f: json.dump([], f, indent=2)

def append_target(path: str, x: float, y: float, z: float):
    ensure_json_array(path)
    with open(path, "r") as f: data = json.load(f)
    data.append({"x": x, "y": y, "z": z})
    with open(path, "w") as f: json.dump(data, f, indent=2)

class App:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title(f"Panda Target GUI (JSON → ROS2)  [{SERVER_NODE}]")
        self.path_var = tk.StringVar(value=JSON_DEFAULT)
        self.x_var = tk.StringVar(value="0.50")
        self.y_var = tk.StringVar(value="0.00")
        self.z_var = tk.StringVar(value="0.50")

        pad = {'padx': 6, 'pady': 4}
        tk.Label(self.root, text="JSON Path").grid(row=0, column=0, sticky="e", **pad)
        tk.Entry(self.root, textvariable=self.path_var, width=60).grid(row=0, column=1, columnspan=3, **pad)

        tk.Label(self.root, text="X").grid(row=1, column=0, sticky="e", **pad)
        tk.Entry(self.root, textvariable=self.x_var, width=10).grid(row=1, column=1, **pad)
        tk.Label(self.root, text="Y").grid(row=1, column=2, sticky="e", **pad)
        tk.Entry(self.root, textvariable=self.y_var, width=10).grid(row=1, column=3, **pad)
        tk.Label(self.root, text="Z").grid(row=1, column=4, sticky="e", **pad)
        tk.Entry(self.root, textvariable=self.z_var, width=10).grid(row=1, column=5, **pad)

        tk.Button(self.root, text="Add to JSON", command=self.on_add).grid(row=2, column=1, **pad)
        tk.Button(self.root, text="Add & Move", command=self.on_add_move).grid(row=2, column=2, **pad)
        tk.Button(self.root, text="Go Home", command=self.on_home).grid(row=2, column=3, **pad)

        self.root.protocol("WM_DELETE_WINDOW", self.root.destroy)

    def parse_xyz(self):
        try:
            return float(self.x_var.get()), float(self.y_var.get()), float(self.z_var.get())
        except ValueError:
            messagebox.showerror("Hata", "X, Y, Z sayı olmalı.")
            return None

    def on_add(self):
        p = self.path_var.get()
        nums = self.parse_xyz()
        if not nums: return
        x, y, z = nums
        try:
            append_target(p, x, y, z)
            messagebox.showinfo("OK", f"JSON’a eklendi: ({x:.3f}, {y:.3f}, {z:.3f})")
        except Exception as e:
            messagebox.showerror("Hata", f"JSON yazma hatası:\n{e}")

    def on_add_move(self):
        svc = f"{SERVER_NODE}/process_last_target_from_json"
        if not wait_service(svc, 30):
            messagebox.showwarning("Uyarı", f"Service '{svc}' bulunamadı (30 sn).")
            return
        p = self.path_var.get()
        nums = self.parse_xyz()
        if not nums: return
        x, y, z = nums
        try:
            append_target(p, x, y, z)
        except Exception as e:
            messagebox.showerror("Hata", f"JSON yazma hatası:\n{e}")
            return
        ok, out = call_ros2(["ros2","service","call", svc, "std_srvs/srv/Trigger", "{}"], timeout=30)
        if ok: messagebox.showinfo("OK","Hedefe gidildi.")
        else:  messagebox.showwarning("Fail", out)

    def on_home(self):
        svc = f"{SERVER_NODE}/go_home"
        if not wait_service(svc, 30):
            messagebox.showwarning("Uyarı", f"Service '{svc}' bulunamadı (30 sn).")
            return
        ok, out = call_ros2(["ros2","service","call", svc, "std_srvs/srv/Trigger", "{}"], timeout=30)
        if ok: messagebox.showinfo("OK","Home OK.")
        else:  messagebox.showwarning("Fail", out)

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    App().run()
