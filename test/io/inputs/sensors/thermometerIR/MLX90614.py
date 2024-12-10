import smbus
import time
import tkinter as tk
from tkinter import StringVar

class MLX90614():
    MLX90614_RAWIR1 = 0x04
    MLX90614_RAWIR2 = 0x05
    MLX90614_TA = 0x06
    MLX90614_TOBJ1 = 0x07
    MLX90614_TOBJ2 = 0x08

    comm_retries = 5
    comm_sleep_amount = 0.1

    def __init__(self, address=0x5a, bus_num=1):
        self.bus_num = bus_num
        self.address = address
        self.bus = smbus.SMBus(bus=bus_num)

    def read_reg(self, reg_addr):
        err = None
        for i in range(self.comm_retries):
            try:
                return self.bus.read_word_data(self.address, reg_addr)
            except IOError as e:
                err = e
                time.sleep(self.comm_sleep_amount)
        raise err

    def data_to_temp(self, data):
        temp = (data * 0.02) - 273.15
        return ((0.986 * temp) + 0.507)

    def get_amb_temp(self):
        data = self.read_reg(self.MLX90614_TA)
        return self.data_to_temp(data)

    def get_obj_temp(self):
        data = self.read_reg(self.MLX90614_TOBJ1)
        return self.data_to_temp(data)

# GUI Setup
def update_temp():
    try:
        obj_temp_c = round(sensor.get_obj_temp(), 1)
        obj_temp_f = round((obj_temp_c * 9 / 5 + 32), 2)

        obj_temp_c_label.set(f"{obj_temp_c} °C")
        obj_temp_f_label.set(f"{obj_temp_f} °F")
    except Exception as e:
        obj_temp_c_label.set("Error")
        obj_temp_f_label.set("Error")
        print(f"Error reading sensor: {e}")

    root.after(500, update_temp)  # Update temperature every 500ms

if __name__ == "__main__":
    sensor = MLX90614()

    # Create the Tkinter window
    root = tk.Tk()
    root.title("Temperature Display")
    root.geometry("800x400")
    root.configure(bg="black")

    # Object Temp Labels
    tk.Label(root, text="Object Temperature", font=("Helvetica", 24), bg="black", fg="white").pack(pady=20)
    
    obj_temp_c_label = StringVar()
    obj_temp_f_label = StringVar()

    tk.Label(root, textvariable=obj_temp_c_label, font=("Helvetica", 48), bg="black", fg="white").pack(pady=10)
    tk.Label(root, textvariable=obj_temp_f_label, font=("Helvetica", 48), bg="black", fg="white").pack(pady=10)

    # Start updating temperatures
    update_temp()

    # Start the Tkinter event loop
    root.mainloop()
