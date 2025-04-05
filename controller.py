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
    level=logging.DEBUG,  # Changed to DEBUG for more detail
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

# Motor class (unchanged from your version)
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
        self.max_speed_value = 300
        
        logging.info(f"Motor {self.name} initialized on pins: EN={enable_pin}, IN1={in1_pin}, IN2={in2_pin}")
    
    def set_speed(self, direction, speed):
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
            self.current_speed = speed
        elif direction == Direction.REVERSE:
            GPIO.output(self.in1_pin, GPIO.LOW)
            GPIO.output(self.in2_pin, GPIO.HIGH)
            self.current_direction = Direction.REVERSE
            self.current_speed = -speed
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

# RobotControl class (simplified and with startup test)
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
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        self.motor_a = Motor("A", self.pin_config['motor_a']['enable'], 
                            self.pin_config['motor_a']['in1'], 
                            self.pin_config['motor_a']['in2'])
        
        self.motor_b = Motor("B", self.pin_config['motor_b']['enable'], 
                            self.pin_config['motor_b']['in3'], 
                            self.pin_config['motor_b']['in4'])
        
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.running = False
        self.control_thread = None
        self.command_stack = deque()
        
        self.start_control_loop()
        self.test_motors()  # Test motors on startup
        logging.info("Robot control system initialized")
    
    def test_motors(self):
        logging.info("Testing motors: moving forward for 2 seconds")
        self.motor_a.set_speed(Direction.FORWARD, 300)
        self.motor_b.set_speed(Direction.FORWARD, 300)
        time.sleep(2)
        self.motor_a.stop()
        self.motor_b.stop()
        logging.info("Motor test complete")
    
    def add_command(self, command, delay):
        timestamp = time.time()
        self.command_stack.append((command, timestamp, delay, delay))
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
            
            updated_stack = deque()
            for command, timestamp, delay, time_left in self.command_stack:
                new_time_left = max(0, time_left - dt)
                if new_time_left > 0:
                    updated_stack.append((command, timestamp, delay, new_time_left))
                else:
                    logging.info(f"Removed expired command: {command}")
            self.command_stack = updated_stack
            
            if self.command_stack:
                latest_command, _, _, time_left = self.command_stack[-1]
                logging.debug(f"Processing command: {latest_command}, Time left: {time_left:.2f}s")
                self._execute_command(latest_command)
            else:
                logging.debug("No commands in stack, stopping motors")
                self.motor_a.stop()
                self.motor_b.stop()
            
            last_time = current_time
            elapsed = time.time() - loop_start
            if elapsed < loop_time:
                time.sleep(loop_time - elapsed)
    
    def _execute_command(self, command):
        action = command.get("action")
        logging.debug(f"Executing command: {action}")
        if action == "accelerate":
            speed = command.get("speed", 300)
            logging.info(f"Accelerate command: setting speed to {speed}")
            self.motor_a.set_speed(Direction.FORWARD, speed)
            self.motor_b.set_speed(Direction.FORWARD, speed)
        elif action == "reverse":
            speed = command.get("speed", 300)
            self.motor_a.set_speed(Direction.REVERSE, speed)
            self.motor_b.set_speed(Direction.REVERSE, speed)
        elif action == "stop":
            self.motor_a.stop()
            self.motor_b.stop()
        elif action == "emergency_stop":
            self.motor_a.brake()
            self.motor_b.brake()
    
    def stop_control_loop(self):
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        logging.info("Control loop stopped")
    
    def accelerate(self, speed=None, delay=5.0, direction="forward"):
        if speed is None:
            speed = 300
        command = {"action": "accelerate", "speed": speed, "direction": direction}
        self.add_command(command, delay)
        return {"status": "success", "action": "accelerate", "speed": speed, "direction": direction, "delay": delay}
    
    def reverse(self, speed=None, delay=5.0):
        if speed is None:
            speed = 300
        command = {"action": "reverse", "speed": speed}
        self.add_command(command, delay)
        return {"status": "success", "action": "reverse", "speed": speed, "delay": delay}
    
    def stop(self, delay=2.0):
        command = {"action": "stop"}
        self.add_command(command, delay)
        return {"status": "success", "action": "stop", "delay": delay}
    
    def emergency_stop(self, delay=1.0):
        command = {"action": "emergency_stop"}
        self.add_command(command, delay)
        return {"status": "success", "action": "emergency_stop", "delay": delay}
    
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
    speed = data.get('speed', 300)
    delay = float(data.get('delay', 5.0))
    result = robot.accelerate(speed, delay)
    return jsonify(result)

@app.route('/reverse', methods=['POST'])
def reverse():
    data = request.get_json() or {}
    speed = data.get('speed', 300)
    delay = float(data.get('delay', 5.0))
    result = robot.reverse(speed, delay)
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

@app.route('/status', methods=['GET'])
def status():
    command_stack_info = [(cmd, timestamp, delay, time_left) for cmd, timestamp, delay, time_left in robot.command_stack]
    return jsonify({
        "status": "running",
        "motor_a_speed": robot.motor_a.current_speed,
        "motor_b_speed": robot.motor_b.current_speed,
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