import time
import subprocess
import smbus2
import os
import signal
import threading
import tkinter as tk
import RPi.GPIO as GPIO
import serial

# === Arduino Serial Setup ===
try:
    arduino = serial.Serial("/dev/ttyACM0", 115200, timeout=1)  # Change if needed
    print("[INFO] Connected to Arduino.")
except Exception as e:
    print(f"[WARNING] Could not connect to Arduino: {e}")
    arduino = None

# === GPIO Setup ===
print("[DEBUG] Setting up GPIO")
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

TRIGGER_OUT_PIN = 17
TRIGGER_IN_PIN = 27

GPIO.setup(TRIGGER_OUT_PIN, GPIO.OUT)
GPIO.setup(TRIGGER_IN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.output(TRIGGER_OUT_PIN, GPIO.HIGH)

# === Device Script Map ===
running_processes = {}

KNOWN_SENSORS = {
    0x57: "/home/tkzisserson/RaspberryPi-Code_24-25/Arduino-Pi Communication/main.py",
    0x5A: "/home/tkzisserson/RaspberryPi-Code_24-25/Arduino-Pi Communication/MLX90614.py"
}

devices_to_scan = list(KNOWN_SENSORS.keys())

def scan_i2c_devices(devices_to_find, max_attempts=10, delay_between_attempts=0.2):
    for _ in range(max_attempts):
        bus = smbus2.SMBus(1)
        found = []
        for address in devices_to_find:
            try:
                bus.write_quick(address)
                found.append(address)
            except:
                pass
        bus.close()
        if found:
            return found
        time.sleep(delay_between_attempts)
    return []

def handle_device(address):
    if address in running_processes:
        print(f"[SKIP] Process for 0x{address:02X} is already running.")
        return

    if address in KNOWN_SENSORS:
        script = KNOWN_SENSORS[address]
        print(f"[INFO] Found 0x{address:02X} → Launching {script}")
        try:
            proc = subprocess.Popen(["python3", script])
            running_processes[address] = proc
        except Exception as e:
            print(f"[ERROR] Failed to launch script for 0x{address:02X}: {e}")
    else:
        print(f"[INFO] Unknown device: 0x{address:02X}")

def killwindow():
    for addr, proc in list(running_processes.items()):
        try:
            proc.terminate()
            proc.wait(timeout=5)
        except Exception as e:
            print(f"[Error] Could not terminate process for 0x{addr:02x}: {e}")
        finally:
            running_processes.pop(addr, None)

def kill_subprocess():
    for address, proc in list(running_processes.items()):
        print(f"[INFO] Attempting to stop process for 0x{address:02X}")
        if proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=5)
                print(f"[SUCCESS] Terminated process for 0x{address:02X}")
            except subprocess.TimeoutExpired:
                print(f"[TIMEOUT] Force killing process for 0x{address:02X}")
                proc.kill()
                proc.wait()
            except Exception as e:
                print(f"[ERROR] Exception killing 0x{address:02X}: {e}")
        else:
            print(f"[INFO] Process for 0x{address:02X} already exited.")
        running_processes.pop(address, None)
    print("[STATUS] All subprocesses cleaned up.")

# === Safe GPIO Monitoring Thread ===
def monitor_gpio():
    print("[INFO] Monitoring GPIO 27 for trigger from GPIO 17...")
    triggered = False

    while True:
        try:
            pin_state = GPIO.input(TRIGGER_IN_PIN)
        except Exception as e:
            print(f"[ERROR] GPIO input failed: {e}")
            print("[INFO] Cleaning up and exiting GPIO monitor thread.")
            GPIO.cleanup()
            return

        if pin_state == GPIO.HIGH and not triggered:
            print("[TRIGGER] GPIO 27 saw HIGH from GPIO 17 → Starting I2C Scan")
            triggered = True
            devices = scan_i2c_devices(devices_to_scan)
            for addr in devices:
                handle_device(addr)
        elif pin_state == GPIO.LOW and triggered:
            print("[RELEASE] GPIO 27 went LOW → Trigger cleanup")
            print("[DEBUG] Processes before kill:", running_processes)
            kill_subprocess()
            print("[DEBUG] Processes after kill:", running_processes)
            triggered = False

        time.sleep(0.1)

# === Key Event Handler for Arduino ===
def send_serial_command(event):
    if event.char in ["1", "2", "3"] and arduino:
        try:
            arduino.write(event.char.encode())
            print(f"[SEND] Sent '{event.char}' to Arduino")
        except Exception as e:
            print(f"[ERROR] Serial write failed: {e}")

# === GUI Setup ===
def run_gui():
    root = tk.Tk()
    root.attributes('-fullscreen', True)
    root.configure(bg='black')

    try:
        photo = tk.PhotoImage(file="leoDefaultFaceFinal.png")
        label = tk.Label(root, image=photo, bg='black')
        label.photo = photo  # prevent garbage collection
        label.pack()
    except Exception as e:
        print(f"[WARNING] Could not load image: {e}")
        tk.Label(root, text="Leo UI", font=("Helvetica", 48), fg="white", bg="black").pack(pady=40)

    root.bind("<e>", lambda e: root.destroy())
    root.bind("<Key>", send_serial_command)

    root.mainloop()

# === MAIN ===
if __name__ == "__main__":
    try:
        threading.Thread(target=monitor_gpio, daemon=True).start()
        run_gui()
    except KeyboardInterrupt:
        print("\n[EXIT] Cleaning up GPIO and exiting.")
    finally:
        GPIO.cleanup()
        if arduino:
            arduino.close()
