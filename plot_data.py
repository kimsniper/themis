import serial
import queue
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Serial settings
PORT = '/dev/ttyUSB0'
BAUD = 115200
ser = serial.Serial(PORT, BAUD, timeout=1)

# Shared queues
out_queue = queue.Queue()
roll_data = []
acc_roll_data = []
gyro_roll_data = []
servo_data = []
max_points = 100

class PIDVisualizer:
    def __init__(self, root):
        self.root = root
        root.title("THEMIS: Real-Time Roll Stabilization")

        self.slider_labels = {}

        # Main frames
        self.control_frame = tk.Frame(root)
        self.control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10)

        self.plot_frame = tk.Frame(root)
        self.plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # Gain vars
        self.kp_var = tk.DoubleVar(value=3.0)
        self.ki_var = tk.DoubleVar(value=0.05)
        self.kd_var = tk.DoubleVar(value=0.5)
        self.alpha_var = tk.DoubleVar(value=0.98)

        # Sliders
        self.add_slider("Kp", 0, 10, self.kp_var, self.set_kp)
        self.add_slider("Ki", 0, 1, self.ki_var, self.set_ki)
        self.add_slider("Kd", 0, 5, self.kd_var, self.set_kd)
        self.add_alpha_slider("Complementary Filter α (Gyro Weight)", 0.0, 1.0, self.alpha_var, self.set_alpha)

        # Terminal
        ttk.Label(self.control_frame, text="Terminal Output").pack(pady=(20, 0))
        self.terminal = ScrolledText(self.control_frame, width=40, height=20, state=tk.DISABLED, font=("Courier", 8))
        self.terminal.pack(fill=tk.BOTH, expand=True, pady=5)

        # Plot setup
        self.fig, (self.ax_roll, self.ax_servo) = plt.subplots(2, 1, figsize=(6, 6), sharex=True)
        self.fig.tight_layout(pad=3.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100)

        # Request gain values from MCU after startup
        self.root.after(2000, self.query_mcu_gains)

    def add_slider(self, label, frm, to, var, command):
        frame = tk.Frame(self.control_frame)
        frame.pack(pady=(10, 0), anchor="w")

        ttk.Label(frame, text=label).pack(side=tk.LEFT)
        slider = ttk.Scale(frame, from_=frm, to=to, orient=tk.HORIZONTAL, length=150, variable=var)
        slider.pack(side=tk.LEFT)

        val_label = ttk.Label(frame, text=f"{var.get():.2f}", width=5)
        val_label.pack(side=tk.LEFT)

        def on_slide(val):
            val_label.config(text=f"{float(val):.2f}")

        def on_release(event):
            command(var.get())

        slider.config(command=on_slide)
        slider.bind("<ButtonRelease-1>", on_release)

        self.slider_labels[label] = val_label  # fix: use string key

    def add_alpha_slider(self, label, frm, to, var, command):
        frame = tk.Frame(self.control_frame)
        frame.pack(pady=(10, 0), anchor="w")

        ttk.Label(frame, text=label).pack(anchor="w")

        slider = ttk.Scale(frame, from_=frm, to=to, orient=tk.HORIZONTAL, length=150, variable=var)
        slider.pack()

        self.alpha_gyro_label = ttk.Label(frame, text=f"Gyro (α): {var.get():.3f}")
        self.alpha_gyro_label.pack(anchor="w")

        self.alpha_accel_label = ttk.Label(frame, text=f"Accel (1−α): {1 - var.get():.3f}")
        self.alpha_accel_label.pack(anchor="w")

        def on_slide(val):
            alpha = float(val)
            self.alpha_gyro_label.config(text=f"Gyro (α): {alpha:.3f}")
            self.alpha_accel_label.config(text=f"Accel (1−α): {1 - alpha:.3f}")

        def on_release(event):
            command(var.get())

        slider.config(command=on_slide)
        slider.bind("<ButtonRelease-1>", on_release)

    def set_kp(self, val): out_queue.put(f"p {val:.2f}\n")
    def set_ki(self, val): out_queue.put(f"i {val:.2f}\n")
    def set_kd(self, val): out_queue.put(f"d {val:.2f}\n")
    def set_alpha(self, val): out_queue.put(f"a {val:.3f}\n")

    def query_mcu_gains(self):
        out_queue.put("get\n")

    def update_slider_label(self, label_name):
        label = self.slider_labels.get(label_name)
        if label:
            if label_name == "Kp":
                label.config(text=f"{self.kp_var.get():.2f}")
            elif label_name == "Ki":
                label.config(text=f"{self.ki_var.get():.2f}")
            elif label_name == "Kd":
                label.config(text=f"{self.kd_var.get():.2f}")

    def log_terminal(self, message):
        self.terminal.config(state=tk.NORMAL)
        self.terminal.insert(tk.END, message + '\n')
        self.terminal.see(tk.END)
        self.terminal.config(state=tk.DISABLED)

    def update_plot(self, _):
        while not out_queue.empty():
            cmd = out_queue.get()
            ser.write(cmd.encode())
            self.log_terminal(f"→ {cmd.strip()}")

        if ser.in_waiting:
            try:
                line = ser.readline().decode().strip()
                if not line:
                    return

                self.log_terminal(line)

                # Gain update from MCU
                if line.startswith("Gains"):
                    try:
                        _, kp, ki, kd, alpha = line.split(',')
                        self.kp_var.set(float(kp))
                        self.ki_var.set(float(ki))
                        self.kd_var.set(float(kd))
                        self.alpha_var.set(float(alpha))

                        # update slider labels as well
                        self.update_slider_label("Kp")
                        self.update_slider_label("Ki")
                        self.update_slider_label("Kd")

                        self.alpha_gyro_label.config(text=f"Gyro (α): {float(alpha):.3f}")
                        self.alpha_accel_label.config(text=f"Accel (1−α): {1 - float(alpha):.3f}")
                        self.log_terminal(f"Synced gains: Kp={kp}, Ki={ki}, Kd={kd}, α={alpha}")
                    except Exception as e:
                        self.log_terminal(f"Gain parse error: {e}")
                    return

                parts = line.split(',')
                if len(parts) == 4:
                    roll_val = float(parts[0])
                    acc_val = float(parts[1])
                    gyro_val = float(parts[2])
                    servo_val = float(parts[3])

                    roll_data.append(roll_val)
                    acc_roll_data.append(acc_val)
                    gyro_roll_data.append(gyro_val)
                    servo_data.append(servo_val)

                    if len(roll_data) > max_points:
                        roll_data.pop(0)
                        acc_roll_data.pop(0)
                        gyro_roll_data.pop(0)
                        servo_data.pop(0)

                    # --- Roll Plot ---
                    self.ax_roll.clear()
                    self.ax_roll.plot(roll_data, label='Filtered Roll (°)', color='blue')
                    self.ax_roll.plot(acc_roll_data, label='Accel Roll (°)', color='green', linestyle='--')
                    self.ax_roll.plot(gyro_roll_data, label='Gyro Roll (°)', color='red', linestyle=':')

                    x = len(roll_data) - 1
                    if x >= 0:
                        self.ax_roll.text(x, roll_data[-1] + 6, f"{roll_data[-1]:.1f}°", color='blue', fontsize=12)
                        self.ax_roll.text(x, acc_roll_data[-1] + 2, f"{acc_roll_data[-1]:.1f}°", color='green', fontsize=12)
                        self.ax_roll.text(x, gyro_roll_data[-1] - 4, f"{gyro_roll_data[-1]:.1f}°", color='red', fontsize=12)

                    self.ax_roll.axhline(y=0, color='gray', linestyle='--', label='Setpoint (0°)')
                    self.ax_roll.set_ylim(-60, 60)
                    self.ax_roll.set_ylabel("Roll (°)")
                    self.ax_roll.set_title("Roll Angle vs Time")
                    self.ax_roll.legend()
                    self.ax_roll.grid(True)

                    # --- Servo Plot ---
                    self.ax_servo.clear()
                    self.ax_servo.plot(servo_data, label='Servo Angle (°)', color='orange')
                    x_servo = len(servo_data) - 1
                    if x_servo >= 0:
                        self.ax_servo.text(x_servo, servo_data[-1] + 5, f"{servo_data[-1]:.1f}°", color='orange', fontsize=12)

                    self.ax_servo.set_ylim(0, 180)
                    self.ax_servo.set_ylabel("Servo (°)")
                    self.ax_servo.set_xlabel("Time (frames)")
                    self.ax_servo.set_title("Servo Output vs Time")
                    self.ax_servo.legend()
                    self.ax_servo.grid(True)

                    self.canvas.draw()
            except Exception as e:
                self.log_terminal(f"Parse error: {e}")

# Run
if __name__ == "__main__":
    print(f"Connected to {PORT} at {BAUD} baud")
    root = tk.Tk()
    app = PIDVisualizer(root)
    root.mainloop()
