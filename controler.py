import RPi.GPIO as GPIO
import time
import logging
import json
import signal
import threading
from enum import Enum
from flask import Flask, request, jsonify

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

# PIDController class (unchanged)
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

# Motor class (unchanged)
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

# RobotControl class modified for Flask
class RobotControl:
    def __init__(self, config_file=None):
        self.pin_config = {
            'motor_a': {'enable': 17, 'in1': 27, 'in2': 22},
            'motor_b': {'enable': 25, 'in3': 23, 'in4': 24}
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
        
        self.pid_a = PIDController(setpoint=0, sample_time=0.05)
        self.pid_b = PIDController(setpoint=0, sample_time=0.05)
        
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.running = False
        self.control_thread = None
        
        self.movement_profiles = {
            "normal": {"accel_rate": 5, "decel_rate": 10, "max_speed": 70},
            "sport": {"accel_rate": 10, "decel_rate": 20, "max_speed": 100},
            "eco": {"accel_rate": 2, "decel_rate": 5, "max_speed": 50}
        }
        self.current_profile = "normal"
        
        self.start_control_loop()
        logging.info("Robot control system initialized")
    
    def get_motor_speed_A(self):
        return self.motor_a.current_speed
    
    def get_motor_speed_B(self):
        return self.motor_b.current_speed
    
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
            
            direction_a = Direction.FORWARD if output_a > 0 else Direction.REVERSE if output_a < 0 else Direction.STOP
            direction_b = Direction.FORWARD if output_b > 0 else Direction.REVERSE if output_b < 0 else Direction.STOP
            
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
    
    def accelerate(self, speed=None):
        if speed is None:
            speed = self.movement_profiles[self.current_profile]["max_speed"]
        self.pid_a.setpoint = speed
        self.pid_b.setpoint = speed
        self.pid_a.reset()
        self.pid_b.reset()
        logging.info(f"Command: Accelerate forward (setpoint={speed})")
        return {"status": "success", "action": "accelerate", "speed": speed}
    
    def reverse(self, speed=None):
        if speed is None:
            speed = self.movement_profiles[self.current_profile]["max_speed"] * 0.7
        self.pid_a.setpoint = -speed
        self.pid_b.setpoint = -speed
        self.pid_a.reset()
        self.pid_b.reset()
        logging.info(f"Command: Reverse (setpoint={-speed})")
        return {"status": "success", "action": "reverse", "speed": speed}
    
    def turn(self, direction, radius=0):
        max_speed = self.movement_profiles[self.current_profile]["max_speed"]
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
        
        self.pid_a.reset()
        self.pid_b.reset()
        logging.info(f"Command: Turn {direction} with radius {radius}")
        return {"status": "success", "action": f"turn_{direction}", "radius": radius}
    
    def stop(self):
        self.pid_a.setpoint = 0
        self.pid_b.setpoint = 0
        self.pid_a.reset()
        self.pid_b.reset()
        self.motor_a.stop()
        self.motor_b.stop()
        logging.info("Command: Stop motors")
        return {"status": "success", "action": "stop"}
    
    def emergency_stop(self):
        self.pid_a.setpoint = 0
        self.pid_b.setpoint = 0
        self.pid_a.reset()
        self.pid_b.reset()
        self.motor_a.brake()
        self.motor_b.brake()
        logging.warning("EMERGENCY STOP ACTIVATED")
        return {"status": "success", "action": "emergency_stop"}
    
    def arc(self, direction, arc_angle, speed=None):
        if speed is None:
            speed = self.movement_profiles[self.current_profile]["max_speed"] * 0.8
        differential = min(100, arc_angle / 90 * 100)
        if direction == 'left':
            self.pid_a.setpoint = speed
            self.pid_b.setpoint = max(speed * (1 - differential/100), 0)
        else:
            self.pid_a.setpoint = max(speed * (1 - differential/100), 0)
            self.pid_b.setpoint = speed
        self.pid_a.reset()
        self.pid_b.reset()
        logging.info(f"Command: {direction} arc (angle={arc_angle}°, speed={speed})")
        return {"status": "success", "action": f"arc_{direction}", "angle": arc_angle, "speed": speed}
    
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
    data = request.get_json()
    speed = data.get('speed') if data else None
    result = robot.accelerate(speed)
    return jsonify(result)

@app.route('/reverse', methods=['POST'])
def reverse():
    data = request.get_json()
    speed = data.get('speed') if data else None
    result = robot.reverse(100)
    return jsonify(result)

@app.route('/turn', methods=['POST'])
def turn():
    data = request.get_json()
    if not data or 'direction' not in data:
        return jsonify({"status": "error", "message": "Direction required"}), 400
    direction = data['direction']
    radius = data.get('radius', 0)
    if direction not in ['left', 'right']:
        return jsonify({"status": "error", "message": "Direction must be 'left' or 'right'"}), 400
    result = robot.turn(direction, radius)
    return jsonify(result)

@app.route('/stop', methods=['POST'])
def stop():
    result = robot.stop()
    return jsonify(result)

@app.route('/emergency_stop', methods=['POST'])
def emergency_stop():
    result = robot.emergency_stop()
    return jsonify(result)

@app.route('/arc', methods=['POST'])
def arc():
    data = request.get_json()
    if not data or 'direction' not in data or 'arc_angle' not in data:
        return jsonify({"status": "error", "message": "Direction and arc_angle required"}), 400
    direction = data['direction']
    arc_angle = data['arc_angle']
    speed = data.get('speed')
    if direction not in ['left', 'right']:
        return jsonify({"status": "error", "message": "Direction must be 'left' or 'right'"}), 400
    result = robot.arc(direction, arc_angle, speed)
    return jsonify(result)

@app.route('/set_profile', methods=['POST'])
def set_profile():
    data = request.get_json()
    if not data or 'profile' not in data:
        return jsonify({"status": "error", "message": "Profile required"}), 400
    profile = data['profile']
    result = robot.set_movement_profile(profile)
    return jsonify(result)

@app.route('/status', methods=['GET'])
def status():
    return jsonify({
        "status": "running",
        "profile": robot.current_profile,
        "motor_a_speed": robot.get_motor_speed_A(),
        "motor_b_speed": robot.get_motor_speed_B(),
        "setpoint_a": robot.pid_a.setpoint,
        "setpoint_b": robot.pid_b.setpoint
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
        app.run(host='0.0.0.0', port=5000, debug=False)
    except Exception as e:
        logging.error(f"Server error: {e}")
    finally:
        shutdown_server()