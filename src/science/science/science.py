import rclpy
from rclpy.node import Node, Subscription
from std_msgs.msg import Float32
import pigpio
import time

# Direction definitions
FORWARD = 1
BACKWARD = 0

# Output pins
AUGER_DRILL_EN = 13 
# The L298N uses two pins to control the direction of the motor
AUGER_DRILL_DIR1 = 19
AUGER_DRILL_DIR2 = 26
AUGER_VERTICAL_STEP = 14
AUGER_VERTICAL_DIR = 15

# Output pins
ACTUATOR_EN = 16
# The L298N uses two pins to control the direction of the motor
ACTUATOR_DIR3 = 20
ACTUATOR_DIR4 = 21

LIMIT_SWITCH_BTN = 10


# Default speed for stepper motors
AUGER_VERT_SPEED = 0.0

# Linear actuator PWM frequency
ACTUATOR_FREQUENCY = 1000

# Number of steps per revolution
STEPPER_SPR = 200
# Time between each step (seconds)
STEPPER_TICK_DELAY = 0.001

# Last time the step pins were toggled
LAST_STEPPER_TICK_TIME = 0
# Last value of the step pin
AUGER_VERTICAL_LAST_VALUE = 0
# Are we currently stepping?
AUGER_VERTICAL_ON = 0


PI: pigpio.pi


def set_motor_speed(speed, pwm, GPIO_DIR):
    if abs(speed) < 0.1:
        speed = 0

    speed = int(speed * 1000000)

    if speed > 0:
        PI.write(GPIO_DIR, 0)
        PI.hardware_PWM(pwm, ACTUATOR_FREQUENCY, speed)
    elif speed < 0:
        PI.write(GPIO_DIR, 1)
        PI.hardware_PWM(pwm, ACTUATOR_FREQUENCY, -speed)
    else:
        PI.hardware_PWM(pwm, ACTUATOR_FREQUENCY, 0)


def set_motor_speed_software(speed, pwm_pin, GPIO_DIR):
    if abs(speed) < 0.1:
        speed = 0

    speed = int(speed * 255)

    if speed > 0:
        PI.write(GPIO_DIR, 0)
        PI.set_PWM_dutycycle(pwm_pin, speed)
    elif speed < 0:
        PI.write(GPIO_DIR, 1)
        PI.set_PWM_dutycycle(pwm_pin, -speed)
    else:
        PI.set_PWM_dutycycle(pwm_pin, 0)


def set_auger_vertical(dir: int):
    global AUGER_VERTICAL_ON

    AUGER_VERTICAL_ON = dir != 0
    PI.write(AUGER_VERTICAL_DIR, dir > 0)

def set_auger_speed(speed: float):
    global AUGER_VERT_SPEED
    
    AUGER_VERT_SPEED = speed

def move_auger_vertical():
    global AUGER_VERT_SPEED

    speed = AUGER_VERT_SPEED

    if abs(speed) < 0.1:
        speed = 0

    # Should be fine, since we're already cutting off everything below 0.1 to exactly 0
    if (speed == 0):
        return

    if (PI.read(LIMIT_SWITCH_BTN) == 1 and speed < 0):
        PI.write(AUGER_VERTICAL_STEP,0);
        return

    PI.write(AUGER_VERTICAL_DIR, 1 if (speed > 0) else 0);
    PI.write(AUGER_VERTICAL_STEP,1);
    # 1000 microseconds
    time.sleep(1000/1000000.0);
    PI.write(AUGER_VERTICAL_STEP,0);
    time.sleep(1000/1000000.0);


def set_auger_drill(speed: float):
    if abs(speed) < 0.1:
        speed = 0

    speed = int(speed * 255)

    PI.write(AUGER_DRILL_DIR1, speed > 0)
    PI.write(AUGER_DRILL_DIR2, speed < 0)

    PI.set_PWM_dutycycle(AUGER_DRILL_EN, abs(speed))

def set_actuator(speed: float):
    if abs(speed) < 0.1:
        speed = 0

    speed = int(speed * 255)

    PI.write(ACTUATOR_DIR3, speed > 0)
    PI.write(ACTUATOR_DIR4, speed < 0)

    PI.set_PWM_dutycycle(ACTUATOR_EN, abs(speed))


def stepper_tick():
    global LAST_STEPPER_TICK_TIME
    global AUGER_VERTICAL_ON
    global AUGER_VERTICAL_LAST_VALUE

    if time.time() - LAST_STEPPER_TICK_TIME < STEPPER_TICK_DELAY:
        return

    LAST_STEPPER_TICK_TIME = time.time()

    if AUGER_VERTICAL_ON:
        AUGER_VERTICAL_LAST_VALUE = not AUGER_VERTICAL_LAST_VALUE
        PI.write(AUGER_VERTICAL_STEP, AUGER_VERTICAL_LAST_VALUE)
    elif AUGER_VERTICAL_LAST_VALUE:
        PI.write(AUGER_VERTICAL_STEP, 0)
        AUGER_VERTICAL_LAST_VALUE = 0


def shutdown():
    set_auger_drill(0)
    set_actuator(0)
    set_auger_vertical(0)

    PI.stop()


class Science(Node):
    auger_vertical_sub: Subscription
    auger_drill_sub: Subscription
    auger_actuate_sub: Subscription

    def __init__(self):
        super().__init__('science')

        self.auger_vertical_sub = self.create_subscription(
            Float32,
            '/science/auger/vertical',
            self.auger_vertical_callback,
            10
        )

        self.auger_drill_sub = self.create_subscription(
            Float32,
            '/science/auger/drill',
            self.auger_drill_callback,
            10
        )

        self.auger_actuate_sub = self.create_subscription(
            Float32,
            '/science/auger/actuate',
            self.auger_actuate_callback,
            10
        )

        self.get_logger().info('Science ready')

    def auger_vertical_callback(self, msg: Float32):
        self.get_logger().info('Auger vertical: %s' % msg.data)
    
        set_auger_speed(msg.data)
        # set_auger_vertical(msg.data)
        # move_auger_vertical(msg.data)

    def auger_drill_callback(self, msg: Float32):
        self.get_logger().info('Auger drill: %s' % msg.data)

        set_auger_drill(msg.data)


    def auger_actuate_callback(self, msg: Float32):
        self.get_logger().info('Auger actuate: %s' % msg.data)

        set_actuator(msg.data)


def main(args=None):
    global PI

    PI = pigpio.pi()

    PI.set_mode(AUGER_DRILL_EN, pigpio.OUTPUT)
    PI.set_mode(AUGER_DRILL_DIR1, pigpio.OUTPUT)
    PI.set_mode(AUGER_DRILL_DIR2, pigpio.OUTPUT)

    PI.set_mode(ACTUATOR_EN, pigpio.OUTPUT)
    PI.set_mode(ACTUATOR_DIR3, pigpio.OUTPUT)
    PI.set_mode(ACTUATOR_DIR4, pigpio.OUTPUT)

    PI.set_mode(LIMIT_SWITCH_BTN, pigpio.INPUT)

    PI.set_mode(AUGER_VERTICAL_STEP, pigpio.OUTPUT)
    PI.set_mode(AUGER_VERTICAL_DIR, pigpio.OUTPUT)

    PI.set_PWM_frequency(AUGER_DRILL_EN, 1000)
    PI.set_PWM_dutycycle(AUGER_DRILL_EN, 0)


    rclpy.init(args=args)

    science = Science()

    try:
        while rclpy.ok():
            rclpy.spin_once(science, timeout_sec=0)
            move_auger_vertical()
            # stepper_tick()
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()
        pass
