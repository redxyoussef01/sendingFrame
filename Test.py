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

# Motor direction enum for better readability
class Direction(Enum):
    FORWARD = 1
    REVERSE = -1
    STOP = 0

class PIDController:
    """Enhanced PID controller with anti-windup and derivative filtering"""
    
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
        
        # Internal state
        self._integral = 0.0
        self._last_error = 0.0
        self._last_output = 0.0
        self._last_time = time.time()
        self._filtered_derivative = 0.0
    
    def reset(self):
        """Reset the PID controller's state"""
        self._integral = 0.0
        self._last_error = 0.0
        self._filtered_derivative = 0.0
        self._last_output = 0.0
        self._last_time = time.time()
        logging.debug("PID controller reset")
    
    def update(self, feedback_value):
        """
        Update PID controller with new feedback value
        
        Args:
            feedback_value: Current measured value from the system
            
        Returns:
            float: Control output value
        """
        now = time.time()
        dt = now - self._last_time
        
        # Only update if enough time has passed
        if dt < self.sample_time:
            return self._last_output
        
        # Calculate error
        error = self.setpoint - feedback_value
        
        # Proportional term
        p_term = self.Kp * error
        
        # Integral term with anti-windup
        if self.anti_windup and abs(self._last_output) >= max(abs(limit) for limit in self.output_limits):
            # Only integrate if we're not saturated
            if (error * self._last_output) < 0:
                self._integral += error * dt
        else:
            self._integral += error * dt
        
        # Apply integral limits
        integral_min, integral_max = self.output_limits
        self._integral = max(min(self._integral, integral_max / self.Ki), integral_min / self.Ki)
        i_term = self.Ki * self._integral
        
        # Derivative term with filtering to reduce noise sensitivity
        derivative = (error - self._last_error) / dt if dt > 0 else 0.0
        self._filtered_derivative = (self.filter_coefficient * derivative + 
                                    (1 - self.filter_coefficient) * self._filtered_derivative)
        d_term = self.Kd * self._filtered_derivative
        
        # Calculate output and apply limits
        output = p_term + i_term + d_term
        output = max(min(output, self.output_limits[1]), self.output_limits[0])
        
        # Update state
        self._last_error = error
        self._last_time = now
        self._last_output = output
        
        logging.debug(f"PID: Error={error:.2f}, P={p_term:.2f}, I={i_term:.2f}, D={d_term:.2f}, Out={output:.2f}")
        return output

    def set_tunings(self, Kp=None, Ki=None, Kd=None):
        """Update PID tuning parameters"""
        if Kp is not None:
            self.Kp = Kp
        if Ki is not None:
            self.Ki = Ki
        if Kd is not None:
            self.Kd = Kd
        logging.debug(f"PID tunings updated: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}")


class Motor:
    """Class for controlling a single DC motor"""
    
    def __init__(self, name, enable_pin, in1_pin, in2_pin, 
                 pwm_freq=100, encoder_callback=None):
        self.name = name
        self.enable_pin = enable_pin
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.pwm_freq = pwm_freq
        self.encoder_callback = encoder_callback
        
        # Set up GPIO pins
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.setup(self.in1_pin, GPIO.OUT)
        GPIO.setup(self.in2_pin, GPIO.OUT)
        
        # Initialize PWM
        self.pwm = GPIO.PWM(self.enable_pin, self.pwm_freq)
        self.pwm.start(0)
        
        # Motor state
        self.current_speed = 0
        self.current_direction = Direction.STOP
        
        # Calibration
        self.calibration_offset = 0
        self.min_speed_threshold = 10  # Minimum PWM value to start moving
        
        logging.info(f"Motor {self.name} initialized on pins: EN={enable_pin}, IN1={in1_pin}, IN2={in2_pin}")
    
    def set_speed(self, direction, speed):
        """
        Set motor speed and direction
        
        Args:
            direction: Direction enum value (FORWARD=1, REVERSE=-1, STOP=0)
            speed: PWM value (0-100)
        """
        # Apply calibration offset
        adjusted_speed = max(0, min(100, speed + self.calibration_offset))
        
        # Apply minimum threshold (dead zone compensation)
        if 0 < adjusted_speed < self.min_speed_threshold:
            adjusted_speed = self.min_speed_threshold
        
        # Ensure we have a valid Direction enum
        if isinstance(direction, int):
            direction = Direction(direction) if direction in [d.value for d in Direction] else Direction.STOP
        
        # Set direction pins
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
        else:  # STOP
            GPIO.output(self.in1_pin, GPIO.LOW)
            GPIO.output(self.in2_pin, GPIO.LOW)
            self.current_direction = Direction.STOP
            self.current_speed = 0
            adjusted_speed = 0
        
        # Set PWM duty cycle
        self.pwm.ChangeDutyCycle(adjusted_speed)
        
        logging.debug(f"Motor {self.name} set: direction={direction}, " 
                      f"raw_speed={speed}, adjusted_speed={adjusted_speed}")
    
    def stop(self):
        """Stop the motor"""
        self.set_speed(Direction.STOP, 0)
    
    def brake(self):
        """Electric brake - short circuit the motor terminals"""
        GPIO.output(self.in1_pin, GPIO.HIGH)
        GPIO.output(self.in2_pin, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(100)  # Full PWM for braking
        self.current_speed = 0
        logging.debug(f"Motor {self.name} brake applied")
    
    def get_speed(self):
        """Get current motor speed (with direction sign)"""
        # In a real system, this would use encoder feedback
        if self.encoder_callback:
            return self.encoder_callback()
        return self.current_speed
    
    def calibrate(self, offset):
        """Set speed calibration offset"""
        self.calibration_offset = offset
        logging.info(f"Motor {self.name} calibration offset set to {offset}")
    
    def set_min_threshold(self, threshold):
        """Set minimum speed threshold (dead zone)"""
        self.min_speed_threshold = max(0, min(50, threshold))
        logging.info(f"Motor {self.name} minimum threshold set to {threshold}")
    
    def cleanup(self):
        """Clean up resources"""
        self.pwm.stop()
        logging.debug(f"Motor {self.name} PWM stopped")


class RobotControl:
    """Main robot control class"""
    
    def __init__(self, config_file=None):
        # Default pin configuration
        self.pin_config = {
            'motor_a': {'enable': 17, 'in1': 27, 'in2': 22},
            'motor_b': {'enable': 25, 'in3': 23, 'in4': 24}
        }
        
        # If config file provided, load it
        if config_file:
            try:
                with open(config_file, 'r') as f:
                    config = json.load(f)
                    self.pin_config = config.get('pin_config', self.pin_config)
                    logging.info(f"Configuration loaded from {config_file}")
            except (json.JSONDecodeError, FileNotFoundError) as e:
                logging.error(f"Failed to load config file: {e}")
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Initialize motors
        self.motor_a = Motor(
            "A", 
            self.pin_config['motor_a']['enable'],
            self.pin_config['motor_a']['in1'],
            self.pin_config['motor_a']['in2'],
            encoder_callback=self.get_motor_speed_A
        )
        
        self.motor_b = Motor(
            "B", 
            self.pin_config['motor_b']['enable'],
            self.pin_config['motor_b']['in3'],
            self.pin_config['motor_b']['in4'],
            encoder_callback=self.get_motor_speed_B
        )
        
        # Initialize PID controllers
        self.pid_a = PIDController(setpoint=0, sample_time=0.05)
        self.pid_b = PIDController(setpoint=0, sample_time=0.05)
        
        # Set up signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Control loop state
        self.running = False
        self.control_thread = None
        
        # Movement profiles for smooth acceleration/deceleration
        self.movement_profiles = {
            "normal": {"accel_rate": 5, "decel_rate": 10, "max_speed": 70},
            "sport": {"accel_rate": 10, "decel_rate": 20, "max_speed": 300},
            "eco": {"accel_rate": 2, "decel_rate": 5, "max_speed": 50}
        }
        self.current_profile = "sport"
        
        logging.info("Robot control system initialized")
    
    def get_motor_speed_A(self):
        """Get Motor A speed - replace with actual encoder readings in real system"""
        return self.motor_a.current_speed
    
    def get_motor_speed_B(self):
        """Get Motor B speed - replace with actual encoder readings in real system"""
        return self.motor_b.current_speed
    
    def signal_handler(self, sig, frame):
        """Handle signals for graceful shutdown"""
        logging.info(f"Received signal {sig}, shutting down...")
        self.stop()
        self.cleanup()
        
    def set_pid_tunings(self, motor="both", Kp=None, Ki=None, Kd=None):
        """Update PID tuning parameters for specified motor(s)"""
        if motor in ["a", "both"]:
            self.pid_a.set_tunings(Kp, Ki, Kd)
        if motor in ["b", "both"]:
            self.pid_b.set_tunings(Kp, Ki, Kd)
        logging.info(f"PID tunings updated for motor(s) {motor}")
    
    def set_movement_profile(self, profile):
        """Set movement profile for robot performance characteristics"""
        if profile in self.movement_profiles:
            self.current_profile = profile
            logging.info(f"Movement profile set to {profile}")
            return True
        logging.warning(f"Unknown profile: {profile}")
        return False
    
    def calibrate_motors(self):
        """Run motor calibration routine"""
        logging.info("Starting motor calibration...")
        
        # Example simple calibration - in reality, would use sensors
        # Test minimum speed needed to start movement
        for speed in range(5, 30, 5):
            self.motor_a.set_speed(Direction.FORWARD, speed)
            self.motor_b.set_speed(Direction.FORWARD, speed)
            time.sleep(0.5)
            # Check if motors are moving (would use encoders in real system)
            # Set minimum threshold just above the point where motors start moving
            self.motor_a.stop()
            self.motor_b.stop()
            time.sleep(0.5)
        
        # Set calibration values
        self.motor_a.set_min_threshold(15)
        self.motor_b.set_min_threshold(15)
        
        # Test for speed differences between motors and set offset
        self.motor_a.calibrate(0)   # Baseline motor
        self.motor_b.calibrate(5)   # Example: Motor B needs 5% more power to match A
        
        logging.info("Motor calibration complete")
    
    def start_control_loop(self):
        """Start the control loop in a separate thread"""
        if self.running:
            logging.warning("Control loop already running")
            return
        
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        logging.info("Control loop started")
    
    def _control_loop(self):
        """Main control loop for PID controllers"""
        loop_time = 0.02  # 50Hz update rate
        
        while self.running:
            loop_start = time.time()
            
            # Get current motor speeds (feedback)
            feedback_a = self.get_motor_speed_A()
            feedback_b = self.get_motor_speed_B()
            
            # Update PID controllers
            output_a = self.pid_a.update(feedback_a)
            output_b = self.pid_b.update(feedback_b)
            
            # Convert PID output to motor commands
            direction_a = Direction.FORWARD if output_a > 0 else Direction.REVERSE if output_a < 0 else Direction.STOP
            direction_b = Direction.FORWARD if output_b > 0 else Direction.REVERSE if output_b < 0 else Direction.STOP
            
            speed_a = min(100, abs(output_a))
            speed_b = min(100, abs(output_b))
            
            # Set motor speeds
            self.motor_a.set_speed(direction_a, speed_a)
            self.motor_b.set_speed(direction_b, speed_b)
            
            # Debug logging (only occasionally to avoid flooding logs)
            if int(time.time() * 5) % 5 == 0:  # Log roughly once per second
                logging.debug(
                    f"Control: Setpoints(A={self.pid_a.setpoint:.1f}, B={self.pid_b.setpoint:.1f}) | "
                    f"Feedback(A={feedback_a:.1f}, B={feedback_b:.1f}) | "
                    f"Output(A={output_a:.1f}, B={output_b:.1f})"
                )
            
            # Sleep to maintain loop rate
            elapsed = time.time() - loop_start
            if elapsed < loop_time:
                time.sleep(loop_time - elapsed)
    
    def stop_control_loop(self):
        """Stop the control loop"""
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        logging.info("Control loop stopped")
    
    # Movement commands
    def reverse(self, speed=None):
        """Accelerate forward at specified speed"""
        if speed is None:
            speed = self.movement_profiles[self.current_profile]["max_speed"]
        
        self.pid_a.setpoint = speed
        self.pid_b.setpoint = speed
        self.pid_a.reset()
        self.pid_b.reset()
        logging.info(f"Command: Accelerate forward (setpoint={speed})")

    def accelerate(self, speed=None):
        """Move in reverse at specified speed"""
        if speed is None:
            speed = self.movement_profiles[self.current_profile]["max_speed"] * 0.7  # Typically slower in reverse
        
        self.pid_a.setpoint = -speed
        self.pid_b.setpoint = -speed
        self.pid_a.reset()
        self.pid_b.reset()
        logging.info(f"Command: Reverse (setpoint={-speed})")
    
    def turn(self, direction, radius=0):
        """
        Turn with specified radius
        
        Args:
            direction: 'left' or 'right'
            radius: turn radius, 0 for spot turn, larger for wider turns
        """
        max_speed = self.movement_profiles[self.current_profile]["max_speed"]
        
        if radius == 0:
            # Spot turn (wheels in opposite directions)
            if direction == 'left':
                self.pid_a.setpoint = max_speed
                self.pid_b.setpoint = -max_speed
            else:
                self.pid_a.setpoint = -max_speed
                self.pid_b.setpoint = max_speed
        else:
            # Turn with radius (one wheel faster than the other)
            inner_speed = max_speed * (1 - min(1, 1/max(1, radius)))
            if direction == 'left':
                self.pid_a.setpoint = max_speed  # Outer wheel
                self.pid_b.setpoint = inner_speed  # Inner wheel
            else:
                self.pid_a.setpoint = inner_speed  # Inner wheel
                self.pid_b.setpoint = max_speed  # Outer wheel
        
        self.pid_a.reset()
        self.pid_b.reset()
        logging.info(f"Command: Turn {direction} with radius {radius}")
    
    def stop(self):
        """Stop movement"""
        self.pid_a.setpoint = 0
        self.pid_b.setpoint = 0
        self.pid_a.reset()
        self.pid_b.reset()
        self.motor_a.stop()
        self.motor_b.stop()
        logging.info("Command: Stop motors")
    
    def emergency_stop(self):
        """Emergency stop with electric braking"""
        self.pid_a.setpoint = 0
        self.pid_b.setpoint = 0
        self.pid_a.reset()
        self.pid_b.reset()
        self.motor_a.brake()
        self.motor_b.brake()
        logging.warning("EMERGENCY STOP ACTIVATED")
    
    def arc(self, direction, arc_angle, speed=None):
        """
        Move in an arc pattern
        
        Args:
            direction: 'left' or 'right'
            arc_angle: angle in degrees (determines sharpness)
            speed: forward speed
        """
        if speed is None:
            speed = self.movement_profiles[self.current_profile]["max_speed"] * 0.8
        
        # Convert angle to wheel speed differential
        # Sharper angles = bigger wheel speed difference
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
    
    def cleanup(self):
        """Clean up resources"""
        self.stop_control_loop()
        self.stop()
        self.motor_a.cleanup()
        self.motor_b.cleanup()
        GPIO.cleanup()
        logging.info("Robot control system shutdown complete")


def demo_sequence(robot):
    """Run a demo sequence of movements"""
    logging.info("Starting demo sequence")
    
    try:
        robot.start_control_loop()
        
        # Optional calibration
        #robot.calibrate_motors()
        
        robot.accelerate(120)
        time.sleep(1.2)
        
        robot.arc('right', 90,  -120)
        time.sleep(2)
       
       

        #robot.arc('right', 45, 100)
        #time.sleep(5)
        
        robot.stop()
        time.sleep(1)
        
  
        
        robot.emergency_stop()
        
    except KeyboardInterrupt:
        logging.info("Demo interrupted by user")
    finally:
        robot.stop_control_loop()
        robot.cleanup()


def main():
    """Main function"""
    logging.info("Starting motor control program")
    
    robot = RobotControl()
    
    demo_sequence(robot)
    
    logging.info("Program completed")


if __name__ == '__main__':
    main()