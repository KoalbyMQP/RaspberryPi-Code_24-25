import sys
sys.path.append(".")

from backend.KoalbyHumanoid.ArduinoSerial import ArduinoSerial
import time

s = ArduinoSerial()
s.send_command("1")
time.sleep(1)
s.send_command("100")
time.sleep(1)

startTime = time.time()
for i in range(100):
    s.send_command("5 5") # get position of motor 5 (right ankle)
    s.read_float()
print(f"Time for read/write positions (average over 100): {(time.time() - startTime)/100}")

time.sleep(10)
while True:
    s.send_command("5 5") # get position of motor 5 (right ankle)
    print(s.read_float())