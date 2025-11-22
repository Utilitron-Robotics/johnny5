from dataclasses import dataclass, field
# Assuming lerobot is available in the environment
try:
    from lerobot.cameras.configs import CameraConfig
except ImportError:
    # Fallback or placeholder if lerobot structure is different
    pass

@dataclass
class Johnny5Config:
    # Jetson Orin / RealSense Configuration
    robot_type: str = "johnny5"
    
    # Motor Bus Ports (Jetson USB tty paths)
    left_bus_port: str = "/dev/ttyUSB0" 
    right_bus_port: str = "/dev/ttyUSB1"
    
    # Vision
    # cameras: dict[str, CameraConfig] = field(default_factory=lambda: {})
    
    # Lift Configuration
    lift_enabled: bool = True