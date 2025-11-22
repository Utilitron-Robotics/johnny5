"""
Voice Interaction Module for Johnny 5.
Ported from alanchelmickjr/whoami.
"""

import logging
import time
import json
from typing import Optional, Dict, Any
import pyttsx3
import speech_recognition as sr

logger = logging.getLogger(__name__)

class Johnny5Voice:
    """
    Voice interaction system for asking names and providing audio feedback
    """

    def __init__(
        self,
        audio_device: Optional[str] = None,
        confidence_threshold: float = 0.7
    ):
        self.confidence_threshold = confidence_threshold
        self.audio_device = audio_device

        # Initialize TTS
        try:
            self.tts_engine = pyttsx3.init()
            self.tts_engine.setProperty('rate', 150)
            self.tts_engine.setProperty('volume', 0.9)
            logger.info("TTS Engine (pyttsx3) initialized")
        except Exception as e:
            logger.error(f"Failed to initialize TTS: {e}")
            self.tts_engine = None

        # Initialize STT
        try:
            self.recognizer = sr.Recognizer()
            self.recognizer.energy_threshold = 300
            self.recognizer.dynamic_energy_threshold = True
            logger.info("Speech Recognition initialized")
        except Exception as e:
            logger.error(f"Failed to initialize STT: {e}")
            self.recognizer = None

    def say(self, text: str, wait: bool = True):
        """Speak text using TTS engine."""
        if not self.tts_engine:
            logger.warning(f"TTS unavailable. Would say: {text}")
            return
        
        try:
            self.tts_engine.say(text)
            if wait:
                self.tts_engine.runAndWait()
        except Exception as e:
            logger.error(f"TTS error: {e}")

    def listen(self, prompt: Optional[str] = None, timeout: float = 5.0) -> Optional[str]:
        """Listen for speech input."""
        if not self.recognizer:
            return None

        if prompt:
            self.say(prompt)
            time.sleep(0.5)

        try:
            with sr.Microphone() as source:
                logger.info("Listening...")
                self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                audio = self.recognizer.listen(source, timeout=timeout)
            
            # Use Google Speech Recognition (Online)
            text = self.recognizer.recognize_google(audio)
            logger.info(f"Recognized: {text}")
            return text.strip()
            
        except sr.UnknownValueError:
            logger.debug("Speech unintelligible")
            return None
        except sr.RequestError as e:
            logger.error(f"STT Service Error: {e}")
            return None
        except Exception as e:
            logger.error(f"Listening error: {e}")
            return None

    def ask_name(self) -> Optional[str]:
        """Interactive flow to get user's name."""
        name = self.listen("Hello! I don't think we've met. What is your name?")
        if name:
            confirm = self.listen(f"Did you say {name}?")
            if confirm and "yes" in confirm.lower():
                self.say(f"Nice to meet you, {name}!")
                return name
        return None