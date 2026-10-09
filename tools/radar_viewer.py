import math
import queue
import sys
import threading
import tkinter as tk

import serial


class RadarViewer:
    def __init__(self, root, port):
        self.root = root
        self.port = port
        self.points = {}
        self.current_cycle = None
        self.events = queue.Queue()
        self.canvas = tk.Canvas(root, width=900, height=650, bg="#07111f")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.status = tk.Label(root, text=f"Connecting to {port}...", anchor="w")
        self.status.pack(fill=tk.X)
        self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.serial_thread.start()
        self.root.after(50, self.process_events)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.draw()

    def read_serial(self):
        try:
            with serial.Serial(self.port, 115200, timeout=1) as connection:
                self.events.put(("status", f"Connected to {self.port} at 115200 baud"))
                while True:
                    line = connection.readline().decode("ascii", errors="ignore").strip()
                    if not line or not line.startswith("RADAR,"):
                        continue
                    fields = line.split(",")
                    if len(fields) != 5:
                        continue
                    cycle = fields[1]
                    angle, distance, valid = map(int, fields[2:])
                    self.events.put(("radar", cycle, angle, distance, valid))
        except (serial.SerialException, OSError) as error:
            self.events.put(("status", f"Serial error: {error}"))

    def process_events(self):
        try:
            while True:
                event = self.events.get_nowait()
                if event[0] == "status":
                    self.status.config(text=event[1])
                else:
                    _, cycle, angle, distance, valid = event
                    if cycle != self.current_cycle:
                        self.current_cycle = cycle
                        self.points.clear()
                    if valid > 0 and distance > 0:
                        self.points[angle] = distance
                    else:
                        self.points.pop(angle, None)
        except queue.Empty:
            pass
        self.draw()
        self.root.after(50, self.process_events)

    def draw(self):
        self.canvas.delete("all")
        width = max(self.canvas.winfo_width(), 900)
        height = max(self.canvas.winfo_height(), 650)
        center_x = width / 2
        center_y = height - 70
        radius = min(width * 0.43, height - 120)
        max_range_cm = 250.0

        for fraction in (0.25, 0.5, 0.75, 1.0):
            r = radius * fraction
            self.canvas.create_arc(
                center_x - r, center_y - r, center_x + r, center_y + r,
                start=0, extent=180, style=tk.ARC, outline="#24415c"
            )
            label = f"{int(max_range_cm * fraction)} cm"
            self.canvas.create_text(
                center_x + 5, center_y - r, text=label,
                fill="#7ea4c4", anchor="sw"
            )

        for angle in (0, 45, 90, 135, 160):
            theta = math.radians(180 - angle)
            end_x = center_x + radius * math.cos(theta)
            end_y = center_y - radius * math.sin(theta)
            self.canvas.create_line(
                center_x, center_y, end_x, end_y,
                fill="#24415c", width=1
            )
            self.canvas.create_text(
                end_x, end_y, text=f"{angle} deg", fill="#9cc4df"
            )

        for angle, distance in self.points.items():
            distance = min(distance, max_range_cm)
            theta = math.radians(180 - angle)
            point_radius = radius * distance / max_range_cm
            x = center_x + point_radius * math.cos(theta)
            y = center_y - point_radius * math.sin(theta)
            self.canvas.create_oval(
                x - 5, y - 5, x + 5, y + 5,
                fill="#ff4f64", outline="#ffd5d9"
            )

        self.canvas.create_oval(
            center_x - 6, center_y - 6, center_x + 6, center_y + 6,
            fill="#58d6ff", outline=""
        )
        cycle = self.current_cycle if self.current_cycle is not None else "-"
        self.canvas.create_text(
            20, 20, text=f"SRF04 RADAR | cycle {cycle} | red = obstacle",
            fill="#e8f4ff", anchor="nw", font=("Segoe UI", 14, "bold")
        )

    def close(self):
        self.root.destroy()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    root = tk.Tk()
    root.title("Fire Robot SRF04 Radar")
    root.geometry("900x700")
    RadarViewer(root, port)
    root.mainloop()


if __name__ == "__main__":
    main()
