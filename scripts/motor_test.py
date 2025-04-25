import Jetson.GPIO as GPIO
import time

# Set BOARD numbering mode
GPIO.setmode(GPIO.BOARD)
# 7, 15, 29, 31, 32, 33
# Define pins
IN1 = 32  # Pin 31 (soc_gpio33_pq6) for IN1
IN2 = 31  # Pin 33 (soc_gpio21_pr0) for IN2

# Setup pins as outputs
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

try:
    # Forward
    print("Forward")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    time.sleep(2)

    # Stop
    print("Stop")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    time.sleep(2)

    # Reverse
    print("Reverse")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    time.sleep(2)

    # Stop
    print("Stop")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    time.sleep(2)

except KeyboardInterrupt:
    print("Interrupted by user")
finally:
    GPIO.cleanup()
