"""
This script connects an Adafruit Metro M4 AirLift Lite microcontroller to an ESP32 via Wi-Fi. The microcontroller uses the MQTT protocol to send messages to control two servos on the robot.

Modules:
- `board`: Defines board-specific pin constants.
- `time`: Implements time delays.
- `digitalio`: Handles digital input/output pin operations.
- `pulseio`: Manages PWM (Pulse Width Modulation) signal generation.
- `adafruit_motor.servo`: Controls servo motors.
- `adafruit_esp32spi`: Interfaces with the ESP32 Wi-Fi module.
- `busio`: Handles SPI (Serial Peripheral Interface) communication.
- `secrets`: Stores sensitive information like Wi-Fi and MQTT credentials.
- `adafruit_minimqtt`: Manages MQTT communication.

Hardware Setup:
- Connect the ESP32 to the microcontroller using SPI.
- Define PWM pins for controlling servos.
- Configure two servos connected to PWM pins.

Functions:
- `move_both()`: Moves both servos in a predefined pattern.
- `move_left()`: Moves only the left servo.
- `move_right()`: Moves only the right servo.
- `connected(client, userdata, flags, rc)`: Callback function for MQTT connection events.
- `disconnected(client, userdata, rc)`: Callback function for MQTT disconnection events.
- `message(client, topic, message)`: Callback function for processing incoming MQTT messages.

Usage:
1. Initialize SPI communication and ESP32 Wi-Fi module.
2. Configure PWM for servos.
3. Connect to Wi-Fi using credentials from the `secrets` module.
4. Set up the MQTT client with the Adafruit IO broker and configure callback functions.
5. Enter a loop to continuously process MQTT messages and control servos based on received commands.

Example MQTT Messages:
- `"left"`: Commands the robot to move the left servo.
- `"right"`: Commands the robot to move the right servo.
- Any other message: Commands the robot to move both servos.

Dependencies:
- Adafruit CircuitPython libraries for ESP32, PWM, Servo, and MQTT.
- Network credentials and Adafruit IO credentials stored in the `secrets` module.
"""

import board
import time
from digitalio import DigitalInOut, Direction
import pulseio
from adafruit_motor import servo
from adafruit_esp32spi import adafruit_esp32spi_wifimanager, adafruit_esp32spi, adafruit_esp32spi_socket as socket
import busio
from secrets import secrets
from adafruit_minimqtt import adafruit_minimqtt as mqtt

cs = DigitalInOut(board.ESP_CS)
busy = DigitalInOut(board.ESP_BUSY)
reset = DigitalInOut(board.ESP_RESET)

spi = busio.SPI(board.SCK, board.MOSI, board.MISO)

esp = adafruit_esp32spi.ESP_SPIcontrol(spi, cs, busy, reset)
wifi = adafruit_esp32spi_wifimanager.ESPSPI_WiFiManager(esp, secrets)

pwm1 = pulseio.PWMOut(board.D8, frequency=50)
pwm2 = pulseio.PWMOut(board.D9, frequency=50)

servo1 = servo.Servo(pwm1)
servo2 = servo.Servo(pwm2)
servo_pause = 1.0

def move_both():
    """
    Moves both servos in a predefined pattern.

    The function performs the following sequence:
    - Moves `servo1` to 90 degrees and `servo2` to 0 degrees, then pauses.
    - Moves `servo1` to 0 degrees and `servo2` to 90 degrees, then pauses.
    - Repeats the above movements 2 times.

    :param: None
    :return: None
    """

    for _ in range(2):
        servo1.angle = 90
        servo2.angle = 0
        time.sleep(servo_pause)
        servo1.angle = 0
        servo2.angle = 90
        time.sleep(servo_pause)

def move_left():
    """
    Moves only the left servo.

    The function performs the following sequence:
    - Sets `servo1` to 90 degrees and pauses.
    - Repeats the movement 2 times.

    :param: None
    :return: None
    """

    for _ in range(2):
        servo1.angle = 90
        time.sleep(servo_pause)

def move_right():
    """
    Moves only the right servo.

    The function performs the following sequence:
    - Sets `servo2` to 90 degrees and pauses.
    - Repeats the movement 2 times.

    :param: None
    :return: None
    """

    for _ in range(2):
        servo2.angle = 90
        time.sleep(servo_pause)

def connected(client, userdata, flags, rc):
    """
    Callback function for MQTT connection events.

    This function prints "connected!" upon successful connection and subscribes to the MQTT topic specified in `secrets["aio_username"] + "/feeds/robot-topic"`.

    :param client: The MQTT client instance.
    :type client: mqtt.Client
    :param userdata: User data passed to the callback (not used here).
    :param flags: Response flags from the broker (not used here).
    :param rc: Connection result code.
    :type rc: int
    :return: None
    """

    print("connected!")
    robot_topic = secrets["aio_username"] + "/feeds/robot-topic"
    client.subscribe(robot_topic)

def disconnected(client, userdata, rc):
    """
    Callback function for MQTT disconnection events.

    This function prints "disconnected :(" when the client disconnects.

    :param client: The MQTT client instance.
    :type client: mqtt.Client
    :param userdata: User data passed to the callback (not used here).
    :param rc: Disconnection result code.
    :type rc: int
    :return: None
    """

    print("disconnected :(")

def message(client, topic, message):
    """
    Callback function for handling incoming MQTT messages.

    This function prints the message received on the subscribed topic and executes different functions based on the message content:
    - Calls `move_left()` if the message contains "left".
    - Calls `move_right()` if the message contains "right".
    - Calls `move_both()` for any other message.

    :param client: The MQTT client instance.
    :type client: mqtt.Client
    :param topic: The MQTT topic the message was received on.
    :type topic: str
    :param message: The message payload received.
    :type message: str
    :return: None
    """
    
    print("message: %s", message)
    
    if "left" in message:
        move_left()
    elif "right" in message:
        move_right()
    else:
        move_both()
    
wifi.connect()

mqtt.set_socket(socket, esp)
mqtt_client = mqtt.MQTT(broker="io.adafruit.com",
                        username=secrets["aio_username"],
                        password=secrets["aio_key"])

mqtt_client.on_connect = connected
mqtt_client.on_disconnect = disconnected
mqtt_client.on_message = message 

mqtt_client.connect()

while True:
    mqtt_client.loop()
    time.sleep(2)
