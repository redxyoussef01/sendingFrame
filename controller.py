import RPi.GPIO as GPIO
import time
import logging
import json
import signal
import threading
from enum import Enum
from flask import Flask, request, jsonify
from collections import deque

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

# PIDController class (modified to support higher speed values)
class PIDController:
    def __init__(self, Kp=1.2, Ki=0.1, Kd=0.05, setpoint=0.0, 
                 sample_time=0.1, output_limits=(-300, 300),  # Modified output limits to allow for 300
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

# Motor class (modified to scale speeds from 0-300 to 0-100 for PWM)
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
        self.max_speed_value = 300  # Maximum speed value allowed in the system
        
        logging.info(f"Motor {self.name} initialized on pins: EN={enable_pin}, IN1={in1_pin}, IN2={in2_pin}")
    
    def set_speed(self, direction, speed):
        # Scale speed from 0-300 to 0-100 for PWM duty cycle
        scaled_speed = (speed / self.max_speed_value) * 100
        adjusted_speed = max(0, min(100, scaled_speed + self.calibration_offset))
        
        if 0 < adjusted_speed < self.min_speed_threshold:
            adjusted_speed = self.min_speed_threshold
        
        if isinstance(direction, int):
            direction = Direction(direction) if direction in [d.value for d in Direction] else Direction.STOP
        
        if direction == Direction.FORWARD:
            GPIO.output(self.in1_pin, GPIO.HIGH)
            GPIO.output(self.in2_pin, GPIO.LOW)
            self.current_direction = Direction.FORWARD
            self.current_speed = speed  # Store the original speed value
        elif direction == Direction.REVERSE:
            GPIO.output(self.in1_pin, GPIO.LOW)
            GPIO.output(self.in2_pin, GPIO.HIGH)
            self.current_direction = Direction.REVERSE
            self.current_speed = -speed  # Store the original speed value with negative sign
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

# Ultrasonic Sensor class (unchanged)
class UltrasonicSensor:
    def __init__(self, name, trigger_pin, echo_pin):
        self.name = name
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        
        GPIO.output(self.trigger_pin, False)
        self.last_distance = 0.0
        logging.info(f"Ultrasonic Sensor {self.name} initialized: Trigger={trigger_pin}, Echo={echo_pin}")
    
    def get_distance(self):
        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00001)  # 10us pulse
        GPIO.output(self.trigger_pin, False)
        
        start_time = time.time()
        while GPIO.input(self.echo_pin) == 0:
            start_time = time.time()
            if time.time() - start_time > 0.02:  # Timeout after 20ms
                return -1
        
        end_time = time.time()
        while GPIO.input(self.echo_pin) == 1:
            end_time = time.time()
            if time.time() - start_time > 0.02:  # Timeout after 20ms
                return -1
        
        pulse_duration = end_time - start_time
        distance = (pulse_duration * 34300) / 2  # Distance in cm
        
        self.last_distance = distance
        logging.debug(f"Ultrasonic {self.name}: Distance={distance:.1f}cm")
        return distance

# RobotControl class (modified for 300 speed)
class RobotControl:
    def __init__(self, config_file=None):
        self.pin_config = {
            'motor_a': {'enable': 17, 'in1': 27, 'in2': 22},
            'motor_b': {'enable': 25, 'in3': 23, 'in4': 24},
            'ir_sensors': {'left': 16, 'right': 20},
            'ultrasonic_sensors': {
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
        
        self.motor_a = Motor("A", self.pin_config['motor_a']['enable'], 
                            self.pin_config['motor_a']['in1'], 
                            self.pin_config['motor_a']['in2'], 
                            encoder_callback=self.get_motor_speed_A)
        
        self.motor_b = Motor("B", self.pin_config['motor_b']['enable'], 
                            self.pin_config['motor_b']['in3'], 
                            self.pin_config['motor_b']['in4'], 
                            encoder_callback=self.get_motor_speed_B)
        
        GPIO.setup(self.pin_config['ir_sensors']['left'], GPIO.IN)
        GPIO.setup(self.pin_config['ir_sensors']['right'], GPIO.IN)
        self.ir_left_pin = self.pin_config['ir_sensors']['left']
        self.ir_right_pin = self.pin_config['ir_sensors']['right']
        
        self.us_front = UltrasonicSensor("front", 
                                       self.pin_config['ultrasonic_sensors']['front']['trigger'],
                                       self.pin_config['ultrasonic_sensors']['front']['echo'])
        self.us_rear = UltrasonicSensor("rear", 
                                      self.pin_config['ultrasonic_sensors']['rear']['trigger'],
                                      self.pin_config['ultrasonic_sensors']['rear']['echo'])
        
        self.pid_a = PIDController(setpoint=0, sample_time=0.05, output_limits=(-300, 300))
        self.pid_b = PIDController(setpoint=0, sample_time=0.05, output_limits=(-300, 300))
        
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.running = False
        self.control_thread = None
        self.obstacle_detected = False
        
        # Command stack with timestamp, delay, and time left
        self.command_stack = deque()  # (command, timestamp, delay, time_left)
        
        self.movement_profiles = {
            "normal": {"accel_rate": 15, "decel_rate": 30, "max_speed": 210},
            "sport": {"accel_rate": 30, "decel_rate": 60, "max_speed": 300},  # Updated to 300
            "eco": {"accel_rate": 6, "decel_rate": 15, "max_speed": 150}
        }
        self.current_profile = "sport"  # Default to sport mode with max speed 300
        
        self.start_control_loop()
        logging.info("Robot control system initialized with command stack, IR for road detection, and Ultrasonic sensors")
    
    def get_motor_speed_A(self):
        return self.motor_a.current_speed
    
    def get_motor_speed_B(self):
        return self.motor_b.current_speed
    
    def is_road_detected(self):
        left_reading = GPIO.input(self.ir_left_pin) == 0
        right_reading = GPIO.input(self.ir_right_pin) == 0
        road_detected = left_reading and right_reading
        logging.debug(f"Road detection: Left={left_reading}, Right={right_reading}, Detected={road_detected}")
        return road_detected
    
    def add_command(self, command, delay):
        timestamp = time.time()
        self.command_stack.append((command, timestamp, delay, delay))  # Initial time_left = delay
        logging.info(f"Added command to stack: {command}, Delay: {delay}s, Stack size: {len(self.command_stack)}")
    
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
        last_time = time.time()
        
        while self.running:
            loop_start = time.time()
            current_time = time.time()
            dt = current_time - last_time
            
            # Update time_left for all commands and remove expired ones
            updated_stack = deque()
            for command, timestamp, delay, time_left in self.command_stack:
                new_time_left = max(0, time_left - dt)
                if new_time_left > 0:
                    updated_stack.append((command, timestamp, delay, new_time_left))
                else:
                    logging.info(f"Removed expired command: {command}")
            self.command_stack = updated_stack
            
            front_distance = self.us_front.get_distance()
            rear_distance = self.us_rear.get_distance()
            
            moving_forward = self.pid_a.setpoint > 0 or self.pid_b.setpoint > 0
            moving_backward = self.pid_a.setpoint < 0 or self.pid_b.setpoint < 0
            
            if (moving_forward and front_distance < 10 and front_distance != -1) or \
               (moving_backward and rear_distance < 10 and rear_distance != -1):
                if not self.obstacle_detected:
                    self.emergency_stop()
                    self.obstacle_detected = True
                    logging.warning(f"Obstacle detected: Front={front_distance:.1f}cm, Rear={rear_distance:.1f}cm")
            else:
                # Road detection check removed as per original code modification
                road_detected = self.is_road_detected()
                if not road_detected:
                    # Just log it but continue driving
                    logging.info("Road not detected, but continuing to drive")
                
                self.obstacle_detected = False
                if self.command_stack:
                    latest_command, _, _, _ = self.command_stack[-1]  # Get the latest command
                    self._execute_command(latest_command)
                else:
                    self._regular_control()
            
            last_time = current_time
            elapsed = time.time() - loop_start
            if elapsed < loop_time:
                time.sleep(loop_time - elapsed)
    
    def _execute_command(self, command):
        action = command.get("action")
        if action == "accelerate":
            # Always use 300 as speed unless explicitly set to something else
            speed = command.get("speed", 300)  # Default to 300
            self.pid_a.setpoint = speed
            self.pid_b.setpoint = speed
        elif action == "reverse":
            # Always use 300 as speed unless explicitly set to something else
            speed = command.get("speed", 300)  # Default to 300
            self.pid_a.setpoint = -speed
            self.pid_b.setpoint = -speed
        elif action.startswith("turn_"):
            direction = action.split("_")[1]
            radius = command.get("radius", 0)
            # Always use 300 as max speed
            max_speed = 300
            if radius == 0:
                if direction == 'left':
                    self.pid_a.setpoint = max_speed
                    self.pid_b.setpoint = -max_speed
                else:
                    self.pid_a.setpoint = -max_speed
                    self.pid_b.setpoint = max_speed
            else:
                inner_speed = max_speed * (1 - min(1, 1/max(1, radius)))
                if direction == 'left':
                    self.pid_a.setpoint = max_speed
                    self.pid_b.setpoint = inner_speed
                else:
                    self.pid_a.setpoint = inner_speed
                    self.pid_b.setpoint = max_speed
        elif action.startswith("arc_"):
            direction = action.split("_")[1]
            arc_angle = command["angle"]
            # Always use 300 as speed unless explicitly set to something else
            speed = command.get("speed", 300)  # Default to 300
            differential = min(100, arc_angle / 90 * 100)
            if direction == 'left':
                self.pid_a.setpoint = speed
                self.pid_b.setpoint = max(speed * (1 - differential/100), 0)
            else:
                self.pid_a.setpoint = max(speed * (1 - differential/100), 0)
                self.pid_b.setpoint = speed
        elif action == "stop":
            self.pid_a.setpoint = 0
            self.pid_b.setpoint = 0
        elif action == "emergency_stop":
            self.pid_a.setpoint = 0
            self.pid_b.setpoint = 0
            self.motor_a.brake()
            self.motor_b.brake()
            return
        
        self.pid_a.reset()
        self.pid_b.reset()
        self._regular_control()
    
    def _regular_control(self):
        feedback_a = self.get_motor_speed_A()
        feedback_b = self.get_motor_speed_B()
        
        output_a = self.pid_a.update(feedback_a)
        output_b = self.pid_b.update(feedback_b)
        
        direction_a = Direction.FORWARD if output_a > 0 else Direction.REVERSE if output_a < 0 else Direction.STOP
        direction_b = Direction.FORWARD if output_b > 0 else Direction.REVERSE if output_b < 0 else Direction.STOP
        
        speed_a = abs(output_a)  # No need to cap at 100 since Motor class now handles scaling
        speed_b = abs(output_b)
        
        self.motor_a.set_speed(direction_a, speed_a)
        self.motor_b.set_speed(direction_b, speed_b)
    
    def stop_control_loop(self):
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        logging.info("Control loop stopped")
    
    def accelerate(self, speed=None, delay=5.0, direction="forward"):
        # Default speed set to 300 if not specified
        if speed is None:
            speed = 300
        command = {"action": "accelerate", "speed": speed, "direction": direction}
        self.add_command(command, delay)
        return {"status": "success", "action": "accelerate", "speed": speed, "direction": direction, "delay": delay}    
    
    def reverse(self, speed=None, delay=5.0):
        # Default speed set to 300 if not specified
        if speed is None:
            speed = 300
        command = {"action": "reverse", "speed": speed}
        self.add_command(command, delay)
        return {"status": "success", "action": "reverse", "speed": speed, "delay": delay}
    
    def turn(self, direction, radius=0, delay=5.0):
        command = {"action": f"turn_{direction}", "radius": radius}
        self.add_command(command, delay)
        return {"status": "success", "action": f"turn_{direction}", "radius": radius, "delay": delay}
    
    def stop(self, delay=2.0):
        command = {"action": "stop"}
        self.add_command(command, delay)
        return {"status": "success", "action": "stop", "delay": delay}
    
    def emergency_stop(self, delay=1.0):
        command = {"action": "emergency_stop"}
        self.add_command(command, delay)
        return {"status": "success", "action": "emergency_stop", "delay": delay}
    
    def arc(self, direction, arc_angle, speed=None, delay=5.0):
        # Default speed set to 300 if not specified
        if speed is None:
            speed = 300
        command = {"action": f"arc_{direction}", "angle": arc_angle, "speed": speed}
        self.add_command(command, delay)
        return {"status": "success", "action": f"arc_{direction}", "angle": arc_angle, "speed": speed, "delay": delay}
    
    def set_movement_profile(self, profile):
        if profile in self.movement_profiles:
            self.current_profile = profile
            logging.info(f"Movement profile set to {profile}")
            return {"status": "success", "profile": profile}
        logging.warning(f"Unknown profile: {profile}")
        return {"status": "error", "message": "Unknown profile"}
    
    def cleanup(self):
        self.stop_control_loop()
        self.stop()
        self.motor_a.cleanup()
        self.motor_b.cleanup()
        GPIO.cleanup()
        logging.info("Robot control system shutdown complete")

# Flask application
app = Flask(__name__)
robot = RobotControl()

@app.route('/accelerate', methods=['POST'])
def accelerate():
    data = request.get_json() or {}
    # Always use 300 as default speed
    speed = data.get('speed', 300)
    delay = float(data.get('delay', 5.0))
    result = robot.accelerate(speed, delay)
    return jsonify(result)

@app.route('/reverse', methods=['POST'])
def reverse():
    data = request.get_json() or {}
    # Always use 300 as default speed
    speed = data.get('speed', 300)
    delay = float(data.get('delay', 5.0))
    result = robot.reverse(speed, delay)
    return jsonify(result)

@app.route('/turn', methods=['POST'])
def turn():
    data = request.get_json() or {}
    if 'direction' not in data:
        return jsonify({"status": "error", "message": "Direction required"}), 400
    direction = data['direction']
    radius = data.get('radius', 0)
    delay = float(data.get('delay', 5.0))
    if direction not in ['left', 'right']:
        return jsonify({"status": "error", "message": "Direction must be 'left' or 'right'"}), 400
    result = robot.turn(direction, radius, delay)
    return jsonify(result)

@app.route('/stop', methods=['POST'])
def stop():
    data = request.get_json() or {}
    delay = float(data.get('delay', 2.0))
    result = robot.stop(delay)
    return jsonify(result)

@app.route('/emergency_stop', methods=['POST'])
def emergency_stop():
    data = request.get_json() or {}
    delay = float(data.get('delay', 1.0))
    result = robot.emergency_stop(delay)
    return jsonify(result)

@app.route('/arc', methods=['POST'])
def arc():
    data = request.get_json() or {}
    if 'direction' not in data or 'arc_angle' not in data:
        return jsonify({"status": "error", "message": "Direction and arc_angle required"}), 400
    direction = data['direction']
    arc_angle = data['arc_angle']
    # Always use 300 as default speed
    speed = data.get('speed', 300)
    delay = float(data.get('delay', 5.0))
    if direction not in ['left', 'right']:
        return jsonify({"status": "error", "message": "Direction must be 'left' or 'right'"}), 400
    result = robot.arc(direction, arc_angle, speed, delay)
    return jsonify(result)

@app.route('/set_profile', methods=['POST'])
def set_profile():
    data = request.get_json() or {}
    if 'profile' not in data:
        return jsonify({"status": "error", "message": "Profile required"}), 400
    profile = data['profile']
    result = robot.set_movement_profile(profile)
    return jsonify(result)

@app.route('/status', methods=['GET'])
def status():
    command_stack_info = [(cmd, timestamp, delay, time_left) for cmd, timestamp, delay, time_left in robot.command_stack]
    return jsonify({
        "status": "running",
        "profile": robot.current_profile,
        "motor_a_speed": robot.get_motor_speed_A(),
        "motor_b_speed": robot.get_motor_speed_B(),
        "setpoint_a": robot.pid_a.setpoint,
        "setpoint_b": robot.pid_b.setpoint,
        "road_detected": robot.is_road_detected(),
        "ir_sensors": {
            "left": {"pin": robot.ir_left_pin, "raw_value": GPIO.input(robot.ir_left_pin)},
            "right": {"pin": robot.ir_right_pin, "raw_value": GPIO.input(robot.ir_right_pin)}
        },
        "ultrasonic_sensors": {
            "front": {
                "distance": robot.us_front.get_distance(),
                "trigger_pin": robot.us_front.trigger_pin,
                "echo_pin": robot.us_front.echo_pin
            },
            "rear": {
                "distance": robot.us_rear.get_distance(),
                "trigger_pin": robot.us_rear.trigger_pin,
                "echo_pin": robot.us_rear.echo_pin
            }
        },
        "obstacle_detected": robot.obstacle_detected,
        "command_stack": [{"command": cmd, "timestamp": timestamp, "delay": delay, "time_left": time_left} 
                          for cmd, timestamp, delay, time_left in command_stack_info]
    })

def shutdown_server():
    robot.cleanup()
    logging.info("Flask server shutting down")

@app.route('/shutdown', methods=['POST'])
def shutdown():
    shutdown_server()
    return jsonify({"status": "success", "message": "Server shutting down"}), 200

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=4002, debug=False)
    except Exception as e:
        logging.error(f"Server error: {e}")
    finally:
        shutdown_server()