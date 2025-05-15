import tkinter as tk
import threading
import time
from tkinter import StringVar
from heartrate_monitor import HeartRateMonitor

# GUI Setup
def update_vitals():
    try:
        bpm = round(sensor.bpm, 1)
        spo2 = round(sensor.spo2, 1)

        bpm_label.set(f"BPM: {bpm}")
        spo2_label.set(f"SpO2: {spo2}%")
    except Exception as e:
        bpm_label.set("Error")
        spo2_label.set("Error")
        print(f"Error reading sensor: {e}")

    if tk._default_root:
        tk._default_root.after(500, update_vitals)  # Update every 500ms

def end_fullscreen(event=None):
    root.attributes("-fullscreen", False)
    root.destroy()  # Close the window completely

def ensure_gui():
    global bpm_label, spo2_label, root

    if not tk._default_root:
        print("[INFO] No GUI found. Creating one.")
        root = tk.Tk()
        root.title("Vitals Display")
        root.attributes("-fullscreen", True)
        root.configure(bg="black")
        root.bind("<e>", end_fullscreen)  # Changed from Escape to 'e'

        tk.Label(root, text="Heart Rate & Oxygen Saturation", font=("Helvetica", 36), bg="black", fg="white").pack(pady=20)

        bpm_label = StringVar(value="BPM: --")
        spo2_label = StringVar(value="SpO2: --%")

        tk.Label(root, textvariable=bpm_label, font=("Helvetica", 64), bg="black", fg="white").pack(pady=10)
        tk.Label(root, textvariable=spo2_label, font=("Helvetica", 64), bg="black", fg="white").pack(pady=10)

        update_vitals()
        root.mainloop()
    else:
        print("[INFO] GUI already running.")

if __name__ == "__main__":
    sensor = HeartRateMonitor(print_raw=False, print_result=True)

    # Start sensor in a separate thread
    sensor.start_sensor()

    ensure_gui()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting")
    finally:
        sensor.stop_sensor()
