from johnny5.voice import Johnny5Voice
import time

def test_voice_interaction():
    print("\n=== Testing Johnny 5 Voice System ===")
    voice = Johnny5Voice()
    
    print("Testing TTS...")
    voice.say("Hello! I am Johnny Five. I am alive!", wait=True)
    time.sleep(1)
    
    print("Testing STT (Ask Name Flow)...")
    # This requires a microphone
    try:
        name = voice.ask_name()
        if name:
            print(f"Success! Learned name: {name}")
        else:
            print("Interaction completed without name learning.")
    except Exception as e:
        print(f"STT Test Failed (No Mic?): {e}")

if __name__ == "__main__":
    test_voice_interaction()