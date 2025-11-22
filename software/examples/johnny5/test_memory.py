from johnny5.memory import GunStorageManager
import time

def test_memory():
    print("\n=== Testing Johnny 5 Memory (Gun.js) ===")
    
    # Initialize Memory
    memory = GunStorageManager(robot_id="johnny5_test")
    
    # 1. Create a Memory
    print("Storing new memory...")
    data = {
        "type": "interaction",
        "person": "Dave",
        "topic": "Hal 9000",
        "sentiment": "concerned"
    }
    mem_id = memory.store_private_memory(data)
    print(f"Stored Memory ID: {mem_id}")
    
    # 2. Retrieve Memory
    print("Retrieving memory...")
    retrieved = memory.retrieve_memory(mem_id)
    
    if retrieved:
        print(f"Successfully retrieved: {retrieved}")
    else:
        print("Failed to retrieve memory.")

    memory.close()

if __name__ == "__main__":
    test_memory()