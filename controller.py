import RPi.GPIO as GPIO
import time
import logging
import json
import signal
import threading
from enum import Enum

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("motor_control.log"),
        logging.StreamHandler()
    ]
)

# Motor direction enum
class Direction(Enum):
    FORWARD = 1
    REVERSE = -1
    STOP = 0

class PIDController:
    def __init__(self, Kp=1.2, Ki=0.1, Kd=0.05, setpoint=0.0, 
                 sample_time=0.1, output_limits=(-100, 100),
                 anti_windup=True, filter_coefficient=0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.sample_time = sample_time
        self.output_limits = output_limits
        self.anti_windup = anti_windup
        self.filter_coefficient = filter_coefficient
        
        self._integral = 0.0
        self._last_error = 0.0
        self._last_output = 0.0
        self._last_time = time.time()
        self._filtered_derivative = 0.0
    
    def reset(self):
        self._integral = 0.0
        self._last_error = 0.0
        self._filtered_derivative = 0.0
        self._last_output = 0.0
        self._last_time = time.time()
        logging.debug("PID controller reset")
    
    def update(self, feedback_value):
        now = time.time()
        dt = now - self._last_time
        if dt < self.sample_time:
            return self._last_output
        
        error = self.setpoint - feedback_value
        p_term = self.Kp * error
        
        if self.anti_windup and abs(self._last_output) >= max(abs(limit) for limit in self.output_limits):
            if (error * self._last_output) < 0:
                self._integral += error * dt
        else:
            self._integral += error * dt
        
        integral_min, integral_max = self.output_limits
        self._integral = max(min(self._integral, integral_max / self.Ki), integral_min / self.Ki)
        i_term = self.Ki * self._integral
        
        derivative = (error - self._last_error) / dt if dt > 0 else 0.0
        self._filtered_derivative = (self.filter_coefficient * derivative + 
                                    (1 - self.filter_coefficient) * self._filtered_derivative)
        d_term = self.Kd * self._filtered_derivative
        
        output = p_term + i_term + d_term
        output = max(min(output, self.output_limits[1]), self.output_limits[0])
        
        self._last_error = error
        self._last_time = now
        self._last_output = output
        
        logging.debug(f"PID: Error={error:.2f}, P={p_term:.2f}, I={i_term:.2f}, D={d_term:.2f}, Out={output:.2f}")
        return output

    def set_tunings(self, Kp=None, Ki=None, Kd=None):
        if Kp is not None:
            self.Kp = Kp
        if Ki is not None:
            self.Ki = Ki
        if Kd is not None:
            self.Kd = Kd
        logging.debug(f"PID tunings updated: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}")

class Motor:
    def __init__(self, name, enable_pin, in1_pin, in2_pin, 
                 pwm_freq=100, encoder_callback=None):
        self.name = name
        self.enable_pin = enable_pin
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.pwm_freq = pwm_freq
        self.encoder_callback = encoder_callback
        
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.setup(self.in1_pin, GPIO.OUT)
        GPIO.setup(self.in2_pin, GPIO.OUT)
        
        self.pwm = GPIO.PWM(self.enable_pin, self.pwm_freq)
        self.pwm.start(0)
        
        self.current_speed = 0
        self.current_direction = Direction.STOP
        
        self.calibration_offset = 0
        self.min_speed_threshold = 10
        
        logging.info(f"Motor {self.name} initialized on pins: EN={enable_pin}, IN1={in1_pin}, IN2={in2_pin}")
    
    def set_speed(self, direction, speed):
        adjusted_speed = max(0, min(100, speed + self.calibration_offset))
        if 0 < adjusted_speed < self.min_speed_threshold:
            adjusted_speed = self.min_speed_threshold
        
        if isinstance(direction, int):
            direction = Direction(direction) if direction in [d.value for d in Direction] else Direction.STOP
        
        if direction == Direction.FORWARD:
            GPIO.output(self.in1_pin, GPIO.HIGH)
            GPIO.output(self.in2_pin, GPIO.LOW)
            self.current_direction = Direction.FORWARD
            self.current_speed = adjusted_speed
        elif direction == Direction.REVERSE:
            GPIO.output(self.in1_pin, GPIO.LOW)
            GPIO.output(self.in2_pin, GPIO.HIGH)
            self.current_direction = Direction.REVERSE
            self.current_speed = -adjusted_speed
        else:
            GPIO.output(self.in1_pin, GPIO.LOW)
            GPIO.output(self.in2_pin, GPIO.LOW)
            self.current_direction = Direction.STOP
            self.current_speed = 0
            adjusted_speed = 0
        
        self.pwm.ChangeDutyCycle(adjusted_speed)
        logging.debug(f"Motor {self.name} set: direction={direction}, raw_speed={speed}, adjusted_speed={adjusted_speed}")
    
    def stop(self):
        self.set_speed(Direction.STOP, 0)
    
    def brake(self):
        GPIO.output(self.in1_pin, GPIO.HIGH)
        GPIO.output(self.in2_pin, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(100)
        self.current_speed = 0
        logging.debug(f"Motor {self.name} brake applied")
    
    def get_speed(self):
        if self.encoder_callback:
            return self.encoder_callback()
        return self.current_speed
    
    def cleanup(self):
        self.pwm.stop()
        logging.debug(f"Motor {self.name} PWM stopped")

class RobotControl:
    def __init__(self, config_file=None):
        self.pin_config = {
            'motor_a': {'enable': 17, 'in1': 27, 'in2': 22},
            'motor_b': {'enable': 25, 'in3': 23, 'in4': 24},
            'ir_sensor': {'left': 16, 'right': 20},  # TCRT5000
            'ultrasonic': {
                'front': {'trigger': 5, 'echo': 6},
                'rear': {'trigger': 13, 'echo': 19}
            }
        }
        
        if config_file:
            try:
                with open(config_file, 'r') as f:
                    config = json.load(f)
                    self.pin_config = config.get('pin_config', self.pin_config)
                    logging.info(f"Configuration loaded from {config_file}")
            except (json.JSONDecodeError, FileNotFoundError) as e:
                logging.error(f"Failed to load config file: {e}")
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup IR sensors
        self.ir_left_pin = self.pin_config['ir_sensor']['left']
        self.ir_right_pin = self.pin_config['ir_sensor']['right']
        GPIO.setup(self.ir_left_pin, GPIO.IN)
        GPIO.setup(self.ir_right_pin, GPIO.IN)
        
        # Setup Ultrasonic sensors
        self.front_trigger = self.pin_config['ultrasonic']['front']['trigger']
        self.front_echo = self.pin_config['ultrasonic']['front']['echo']
        self.rear_trigger = self.pin_config['ultrasonic']['rear']['trigger']
        self.rear_echo = self.pin_config['ultrasonic']['rear']['echo']
        GPIO.setup(self.front_trigger, GPIO.OUT)
        GPIO.setup(self.front_echo, GPIO.IN)
        GPIO.setup(self.rear_trigger, GPIO.OUT)
        GPIO.setup(self.rear_echo, GPIO.IN)
        
        # Initialize motors
        self.motor_a = Motor("A", self.pin_config['motor_a']['enable'], 
                            self.pin_config['motor_a']['in1'], 
                            self.pin_config['motor_a']['in2'], 
                            encoder_callback=self.get_motor_speed_A)
        self.motor_b = Motor("B", self.pin_config['motor_b']['enable'], 
                            self.pin_config['motor_b']['in3'], 
                            self.pin_config['motor_b']['in4'], 
                            encoder_callback=self.get_motor_speed_B)
        
        # PID controllers
        self.pid_a = PIDController(setpoint=0, sample_time=0.05)
        self.pid_b = PIDController(setpoint=0, sample_time=0.05)
        self.line_pid = PIDController(Kp=30.0, Ki=0.1, Kd=5.0, setpoint=0, 
                                    sample_time=0.02, output_limits=(-50, 50))
        
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.running = False
        self.control_thread = None
        self.line_following_active = False
        
        self.movement_profiles = {
            "normal": {"accel_rate": 5, "decel_rate": 10, "max_speed": 70},
            "sport": {"accel_rate": 10, "decel_rate": 20, "max_speed": 150},
            "eco": {"accel_rate": 2, "decel_rate": 5, "max_speed": 50}
        }
        self.current_profile = "sport"
        
        self.start_control_loop()
        logging.info("Robot control system initialized with IR and Ultrasonic sensors")
    
    def get_motor_speed_A(self):
        return self.motor_a.current_speed
    
    def get_motor_speed_B(self):
        return self.motor_b.current_speed
    
    def read_ir_sensors(self):
        left = GPIO.input(self.ir_left_pin)
        right = GPIO.input(self.ir_right_pin)
        # TCRT5000: 0 = line detected, 1 = no line
        if left == 0 and right == 0:  # Both on line
            return 0.0
        elif left == 0 and right == 1:  # Left on, right off
            return -0.5
        elif left == 1 and right == 0:  # Right on, left off
            return 0.5
        else:  # Both off
            return None
    
    def get_distance(self, trigger_pin, echo_pin):
        # Send 10us pulse to trigger
        GPIO.output(trigger_pin, True)
        time.sleep(0.00001)
        GPIO.output(trigger_pin, False)
        
        start_time = time.time()
        stop_time = time.time()
        
        while GPIO.input(echo_pin) == 0:
            start_time = time.time()
            if start_time - stop_time > 0.1:  # Timeout
                return None
        
        while GPIO.input(echo_pin) == 1:
            stop_time = time.time()
            if stop_time - start_time > 0.1:  # Timeout
                return None
        
        elapsed = stop_time - start_time
        distance = (elapsed * 34300) / 2  # Speed of sound in cm/s
        return distance
    
    def check_obstacles(self):
        front_dist = self.get_distance(self.front_trigger, self.front_echo)
        rear_dist = self.get_distance(self.rear_trigger, self.rear_echo)
        return (front_dist is not None and front_dist < 5) or (rear_dist is not None and rear_dist < 5)
    
    def signal_handler(self, sig, frame):
        logging.info(f"Received signal {sig}, shutting down...")
        self.stop()
        self.cleanup()
    
    def start_control_loop(self):
        if self.running:
            logging.warning("Control loop already running")
            return
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        logging.info("Control loop started")
    
    def _control_loop(self):
        loop_time = 0.02
        while self.running:
            loop_start = time.time()
            
            feedback_a = self.get_motor_speed_A()
            feedback_b = self.get_motor_speed_B()
            
            output_a = self.pid_a.update(feedback_a)
            output_b = self.pid_b.update(feedback_b)
            
            if self.line_following_active:
                line_error = self.read_ir_sensors()
                if line_error is None or self.check_obstacles():
                    self.motor_a.stop()
                    self.motor_b.stop()
                    if line_error is None:
                        logging.warning("Line lost - stopping")
                    if self.check_obstacles():
                        logging.warning("Obstacle detected within 5cm - stopping")
                else:
                    correction = self.line_pid.update(line_error)
                    base_speed = min(abs(output_a), abs(output_b))
                    output_a = base_speed + correction
                    output_b = base_speed - correction
                    output_a = max(min(output_a, 100), 0)
                    output_b = max(min(output_b, 100), 0)
            
            direction_a = Direction.FORWARD if output_a > 0 else Direction.STOP
            direction_b = Direction.FORWARD if output_b > 0 else Direction.STOP
            
            speed_a = min(100, abs(output_a))
            speed_b = min(100, abs(output_b))
            
            self.motor_a.set_speed(direction_a, speed_a)
            self.motor_b.set_speed(direction_b, speed_b)
            
            elapsed = time.time() - loop_start
            if elapsed < loop_time:
                time.sleep(loop_time - elapsed)
    
    def stop_control_loop(self):
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        logging.info("Control loop stopped")
    
    def start_line_following(self, speed=None):
        if speed is None:
            speed = self.movement_profiles[self.current_profile]["max_speed"] * 0.7
        self.pid_a.setpoint = speed
        self.pid_b.setpoint = speed
        self.line_pid.reset()
        self.line_following_active = True
        logging.info(f"Line following started with speed {speed}")
    
    def stop(self):
        self.pid_a.setpoint = 0
        self.pid_b.setpoint = 0
        self.pid_a.reset()
        self.pid_b.reset()
        self.motor_a.stop()
        self.motor_b.stop()
        self.line_following_active = False
        logging.info("Command: Stop motors")
    
    def accelerate(self, speed=None):
        self.start_line_following(speed)
    
    def reverse(self, speed=None):
        logging.warning("Reverse not supported in line-following mode")
    
    def turn(self, direction, radius=0):
        logging.warning("Manual turn not supported in line-following mode")
    
    def emergency_stop(self):
        self.pid_a.setpoint = 0
        self.pid_b.setpoint = 0
        self.pid_a.reset()
        self.pid_b.reset()
        self.motor_a.brake()
        self.motor_b.brake()
        self.line_following_active = False
        logging.warning("EMERGENCY STOP ACTIVATED")
    
    def arc(self, direction, arc_angle, speed=None):
        logging.warning("Arc not supported in line-following mode")
    
    def set_movement_profile(self, profile):
        if profile in self.movement_profiles:
            self.current_profile = profile
            logging.info(f"Movement profile set to {profile}")
            return True
        logging.warning(f"Unknown profile: {profile}")
        return False
    
    def cleanup(self):
        self.stop_control_loop()
        self.stop()
        self.motor_a.cleanup()
        self.motor_b.cleanup()
        GPIO.cleanup()
        logging.info("Robot control system shutdown complete")

def demo_sequence(robot):
    logging.info("Starting demo sequence")
    try:
        robot.start_control_loop()
        robot.start_line_following(50)
        time.sleep(20)  # Run for 20 seconds
        robot.stop()
    except KeyboardInterrupt:
        logging.info("Demo interrupted by user")
    finally:
        robot.stop_control_loop()
        robot.cleanup()

def main():
    logging.info("Starting motor control program")
    robot = RobotControl()
    demo_sequence(robot)
    logging.info("Program completed")

if __name__ == '__main__':
    main()