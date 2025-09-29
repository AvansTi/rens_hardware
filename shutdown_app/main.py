#!/usr/bin/env python3
"""
GPIO Shutdown Controller for Raspberry Pi 4
Monitors a button on a GPIO pin and initiates shutdown when pressed.
Controls an LED to indicate shutdown status.
"""
"""
1: install python3-rpi.gpio
sudo apt install python3-rpi.gpio

2: run as root:
sudo python3 main.py

3: remove the comment on the line doing the real shutdown ;)
"""
import RPi.GPIO as GPIO
import time
import os
import threading

# GPIO Pin Configuration
BUTTON_PIN = 27  # GPIO pin for shutdown button (change as needed)  SIG2 on Grove Board
LED_PIN = 17     # GPIO pin for status LED (change as needed)       SIG2 on Grove Board

# LED blink interval during shutdown (seconds)
BLINK_INTERVAL = 0.5

# Debounce time for button (seconds)
DEBOUNCE_TIME = 0.2

# Global flag for shutdown state
shutdown_initiated = False
last_button_press = 0


def setup_gpio():
    """Initialize GPIO pins"""
    # Clean up any previous GPIO settings
    GPIO.setwarnings(False)
    GPIO.cleanup()
    
    GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
    
    # Setup button pin with pull-down resistor
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    # Setup LED pin as output
    GPIO.setup(LED_PIN, GPIO.OUT)
    GPIO.output(LED_PIN, GPIO.LOW)  # LED off initially
    
    print(f"GPIO initialized - Button: GPIO{BUTTON_PIN}, LED: GPIO{LED_PIN}")


def blink_led():
    """Blink the LED to indicate shutdown in progress"""
    global shutdown_initiated
    
    while shutdown_initiated:
        try:
            GPIO.output(LED_PIN, GPIO.HIGH)
            time.sleep(BLINK_INTERVAL)
            GPIO.output(LED_PIN, GPIO.LOW)
            time.sleep(BLINK_INTERVAL)
        except:
            break


def shutdown_system():
    """Initiate system shutdown with LED blinking"""
    global shutdown_initiated
    
    print("Shutdown button pressed!")
    print("Initiating system shutdown...")
    
    shutdown_initiated = True
    
    # Start LED blinking in a separate thread
    blink_thread = threading.Thread(target=blink_led, daemon=True)
    blink_thread.start()
    
    # Wait a moment for visual feedback
    time.sleep(2)
    
    # Execute shutdown command
    print("Executing shutdown now...")
    #os.system("sudo shutdown -h now")


def check_button():
    """Check if button is pressed with debouncing"""
    global last_button_press
    
    current_time = time.time()
    
    if GPIO.input(BUTTON_PIN) == GPIO.LOW:
        # Check if enough time has passed since last press (debounce)
        if current_time - last_button_press > DEBOUNCE_TIME:
            last_button_press = current_time
            return True
    
    return False


def main():
    """Main program loop"""
    try:
        setup_gpio()
        
        print("Shutdown monitor running...")
        print(f"Press button on GPIO{BUTTON_PIN} to shutdown")
        print("Press Ctrl+C to exit")
        
        # Poll the button state
        while True:
            if check_button():
                shutdown_system()
                # Keep running until actual shutdown
                while True:
                    time.sleep(1)
            
            time.sleep(0.1)  # Check button every 100ms
            
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("Cleaning up GPIO...")
        GPIO.cleanup()
        print("Program terminated")


if __name__ == "__main__":
    main()
