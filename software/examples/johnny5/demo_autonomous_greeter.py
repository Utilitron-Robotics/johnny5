import time
import logging
from johnny5.robot import Johnny5Robot
from johnny5.config import Johnny5Config

# Configure logging
logging.basicConfig(level=logging.INFO)

def run_demo():
    print("\n=== Johnny 5 Autonomous Greeter Demo ===")
    
    # Initialize Robot
    config = Johnny5Config()
    robot = Johnny5Robot(config)
    
    try:
        robot.connect()
        
        print("\n--- Entering Autonomous Mode ---")
        print("Press Ctrl+C to stop.")
        
        while True:
            # The step function handles the intelligence loop:
            # 1. Sense (Vision/Encoders)
            # 2. Think (LeRobot Policy + WhoAmI Memory)
            # 3. Act (Motor Commands + Voice)
            robot.step()
            
            # Sleep to maintain control loop frequency
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopping demo...")
    finally:
        robot.disconnect()

if __name__ == "__main__":
    run_demo()