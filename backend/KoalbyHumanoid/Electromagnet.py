import RPi.GPIO as GPIO
from enum import Enum

ELECTROMAGNET_GPIO_PIN = 14

class ElectromagnetState(Enum):
    OFF = 0
    ON = 1

class Electromagnet():
    def __init__(self, pin=ELECTROMAGNET_GPIO_PIN):
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        self.state = ElectromagnetState.OFF
        self.turnOff() # off by default
        
    def turnOn(self):
        GPIO.output(self.pin, GPIO.LOW)
        self.state = ElectromagnetState.ON
        
    def turnOff(self):
        GPIO.output(self.pin, GPIO.HIGH) 
        self.state = ElectromagnetState.OFF
        
    def toggle(self):
        if self.state == ElectromagnetState.OFF:
            self.turnOn()
        else:
            self.turnOff()
        