import Jetson.GPIO as GPIO
import keyboard
import time

# Pin numbers (BOARD mode: physical 40-pin header numbers)
PINS = {
    "left_forward": 29,   # Pin 29
    "left_backward": 31,  # Pin 31
    "right_forward": 32,  # Pin 32
    "right_backward": 33  # Pin 33
}

def setup_gpio():
    # Set pin numbering mode to BOARD
    GPIO.setmode(GPIO.BOARD)
    # Setup pins as outputs
    for pin in PINS.values():
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    return PINS

def stop_motors(pins):
    # Set all pins low to stop motors
    for pin in pins.values():
        GPIO.output(pin, GPIO.LOW)

def main():
    print("Use W (Forward), S (Backward), A (Left), D (Right), Q (Quit)")

    # Setup GPIO
    try:
        pins = setup_gpio()
    except Exception as e:
        print(f"GPIO Setup Error: {e}")
        return

    running = True
    while running:
        try:
            # Check key states
            if keyboard.is_pressed('w'):
                GPIO.output(pins["left_forward"], GPIO.HIGH)
                GPIO.output(pins["left_backward"], GPIO.LOW)
                GPIO.output(pins["right_forward"], GPIO.HIGH)
                GPIO.output(pins["right_backward"], GPIO.LOW)
                print("Moving Forward")
            elif keyboard.is_pressed('s'):
                GPIO.output(pins["left_forward"], GPIO.LOW)
                GPIO.output(pins["left_backward"], GPIO.HIGH)
                GPIO.output(pins["right_forward"], GPIO.LOW)
                GPIO.output(pins["right_backward"], GPIO.HIGH)
                print("Moving Backward")
            elif keyboard.is_pressed('a'):
                GPIO.output(pins["left_forward"], GPIO.LOW)
                GPIO.output(pins["left_backward"], GPIO.HIGH)
                GPIO.output(pins["right_forward"], GPIO.HIGH)
                GPIO.output(pins["right_backward"], GPIO.LOW)
                print("Turning Left")
            elif keyboard.is_pressed('d'):
                GPIO.output(pins["left_forward"], GPIO.HIGH)
                GPIO.output(pins["left_backward"], GPIO.LOW)
                GPIO.output(pins["right_forward"], GPIO.LOW)
                GPIO.output(pins["right_backward"], GPIO.HIGH)
                print("Turning Right")
            elif keyboard.is_pressed('q'):
                stop_motors(pins)
                print("Stopping and Quitting")
                running = False
            else:
                stop_motors(pins)

            time.sleep(0.01)  # Small delay to prevent excessive CPU usage

        except Exception as e:
            stop_motors(pins)
            print(f"Error: {e}")
            break

    # Cleanup GPIO
    stop_motors(pins)
    GPIO.cleanup()

if __name__ == "__main__":
    main()