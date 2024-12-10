import tkinter as tk
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

    root.after(500, update_vitals)  # Update every 500ms

if __name__ == "__main__":
    sensor = HeartRateMonitor(print_raw=False, print_result=True)

    # Start sensor in a separate thread
    sensor.start_sensor()

    # Create the Tkinter window
    root = tk.Tk()
    root.title("Vitals Display")
    root.geometry("800x400")
    root.configure(bg="black")

    # Labels for BPM and SpO2
    tk.Label(root, text="Heart Rate & Oxygen Saturation", font=("Helvetica", 24), bg="black", fg="white").pack(pady=20)

    bpm_label = StringVar()
    spo2_label = StringVar()

    tk.Label(root, textvariable=bpm_label, font=("Helvetica", 48), bg="black", fg="white").pack(pady=10)
    tk.Label(root, textvariable=spo2_label, font=("Helvetica", 48), bg="black", fg="white").pack(pady=10)

    # Start updating vitals
    update_vitals()

    # Start the Tkinter event loop
    root.mainloop()

    # Stop the sensor when the GUI closes
    sensor.stop_sensor()
