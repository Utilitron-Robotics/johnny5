import logging
import time
import numpy as np
from typing import Dict, Any

from .config import Johnny5Config
from .memory import GunStorageManager
from .voice import Johnny5Voice
from .vision import Johnny5Vision
from .base import DifferentialDriveBase

# LeRobot Imports (Simulated/Placeholder if dependencies missing)
try:
    from lerobot.common.policies.act.modeling_act import ACTPolicy
    from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
    LEROBOT_AVAILABLE = True
except ImportError:
    LEROBOT_AVAILABLE = False
    logging.warning("LeRobot not fully installed. Running in simulation mode.")

class Johnny5Robot:
    """
    Johnny 5 - Autonomous Mobile Manipulator
    
    Architecture:
    - **Brain**: Jetson Orin Nano (Host)
    - **Eyes**: OAK-D / RealSense (Vision + Depth)
    - **Voice**: PyTTSx3 + SpeechRecognition
    - **Memory**: Gun.js Encrypted Distributed Storage
    - **Body**: Differential Drive Base + Hanging Shoulder Arms
    - **Control**: Hugging Face LeRobot (ACT/Diffusion Policies)
    """
    
    def __init__(self, config: Johnny5Config):
        self.config = config
        self.is_connected = False
        logging.info(f"Initializing Johnny 5 Robot (Type: {config.robot_type})")
        
        # 1. Initialize WhoAmI Intelligence Stack
        self.memory = GunStorageManager(robot_id="johnny5")
        self.voice = Johnny5Voice()
        self.vision = Johnny5Vision()
        
        # Initialize Base
        self.base = DifferentialDriveBase()
        
        logging.info("WhoAmI Intelligence & Hardware Initialized")

        # 2. Initialize LeRobot Policy (Placeholder)
        self.policy = None
        if LEROBOT_AVAILABLE:
            # Example: Loading a pre-trained ACT policy for manipulation
            # self.policy = ACTPolicy.from_pretrained("lerobot/johnny5-manipulation")
            pass

    def connect(self, calibrate: bool = False):
        """Connect to underlying hardware (Motors, Base, Cameras)"""
        if self.is_connected:
            return
            
        logging.info("Connecting to Johnny 5 Hardware...")
        
        # Connect Base
        self.base.connect()
        
        # TODO: Initialize FeetechMotorsBus for Arms
        # self.arm_bus = FeetechMotorsBus(port=self.config.left_bus_port, ...)
        
        self.is_connected = True
        # self.voice.say("Johnny Five is alive!")
        logging.info("Johnny 5 Connected.")

    def disconnect(self):
        """Safe shutdown of all hardware"""
        if not self.is_connected:
            return
            
        logging.info("Disconnecting Johnny 5...")
        self.voice.say("Disassembling...")
        self.is_connected = False

    def get_observation(self) -> Dict[str, Any]:
        """
        Capture sensory data for the LeRobot policy.
        Returns a dictionary compatible with LeRobot dataset formats.
        """
        # 1. Capture Image
        # frame = self.camera.get_frame()
        frame = np.zeros((480, 640, 3), dtype=np.uint8) # Placeholder
        
        # 2. Process Intelligence (WhoAmI)
        faces = self.vision.process_frame(frame)
        for face in faces:
            if face.name != "Unknown":
                mem = self.memory.retrieve_memory(f"person_{face.name}")
                if mem:
                    logging.info(f"Recognized {face.name}, last seen: {mem.get('last_seen')}")

        # 3. Return State Dict (Standardized for Aloha Node)
        # TODO: Read actual arm joints from FeetechBus
        obs = {
            "observation.images.camera_front": frame,
            "lift_axis.height_mm": 0.0,
            "x.vel": 0.0,
            "y.vel": 0.0,
            "theta.vel": 0.0,
        }
        
        # Add dummy arm joints for ROS 2 visualization compatibility
        joint_names = [
            "arm_left_shoulder_pan", "arm_left_shoulder_lift", "arm_left_elbow_flex",
            "arm_left_wrist_flex", "arm_left_wrist_roll", "arm_left_gripper",
            "arm_right_shoulder_pan", "arm_right_shoulder_lift", "arm_right_elbow_flex",
            "arm_right_wrist_flex", "arm_right_wrist_roll", "arm_right_gripper"
        ]
        for name in joint_names:
            obs[f"{name}.pos"] = 0.0
            
        return obs

    def step(self):
        """
        Main Control Loop (The "Heartbeat")
        """
        if not self.is_connected:
            return

        # 1. Sense
        obs = self.get_observation()
        
        # 2. Think (LeRobot Policy Inference)
        action = None
        if self.policy:
            # action = self.policy.select_action(obs)
            pass
            
        # 3. Act
        if action:
            self.send_action(action)
        
        # 4. Interact (Voice/Social Layer)
        # Simple interaction loop example
        # if social_cue_detected:
        #     self.voice.ask_name()

    def send_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """Execute control command to motors"""
        if not self.is_connected:
            return {}

        # Handle Base Movement
        # Action comes in with 'x.vel' (m/s) and 'theta.vel' (deg/s from aloha node)
        if "x.vel" in action and "theta.vel" in action:
            x_vel = float(action["x.vel"])
            theta_deg_s = float(action["theta.vel"])
            
            # Convert deg/s back to rad/s for DifferentialDriveBase
            theta_rad_s = theta_deg_s * (np.pi / 180.0)
            
            self.base.drive(x_vel, theta_rad_s)
            
        # TODO: Handle Arm Joints
        # keys like "arm_left_shoulder_pan.pos" -> send to Feetech bus
        
        return action