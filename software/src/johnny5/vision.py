"""
Face Recognition API for Johnny 5.
Ported from alanchelmickjr/whoami.
"""

import cv2
import numpy as np
import face_recognition
import pickle
import os
import logging
import threading
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class RecognitionResult:
    name: str
    confidence: float
    location: Tuple[int, int, int, int]
    encoding: Optional[np.ndarray] = None

class FaceDatabaseManager:
    """Manages face encodings database."""
    def __init__(self, db_path: str = "face_db.pkl"):
        self.db_path = db_path
        self.encodings: List[np.ndarray] = []
        self.names: List[str] = []
        self.load()

    def add_face(self, name: str, encoding: np.ndarray):
        self.names.append(name)
        self.encodings.append(encoding)
        self.save()

    def save(self):
        with open(self.db_path, 'wb') as f:
            pickle.dump({'names': self.names, 'encodings': self.encodings}, f)

    def load(self):
        if os.path.exists(self.db_path):
            with open(self.db_path, 'rb') as f:
                data = pickle.load(f)
                self.names = data.get('names', [])
                self.encodings = data.get('encodings', [])

class Johnny5Vision:
    """Main vision interface for Johnny 5."""
    
    def __init__(self, db_path: str = "johnny5_faces.pkl"):
        self.db = FaceDatabaseManager(db_path)
        self.process_every_n_frames = 2
        self.frame_count = 0

    def process_frame(self, frame: np.ndarray) -> List[RecognitionResult]:
        """Detect and recognize faces in frame."""
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return []

        # Convert to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Detect faces
        face_locations = face_recognition.face_locations(rgb_frame)
        if not face_locations:
            return []

        # Encode faces
        face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
        
        results = []
        for location, encoding in zip(face_locations, face_encodings):
            name = "Unknown"
            confidence = 0.0
            
            if self.db.encodings:
                matches = face_recognition.compare_faces(self.db.encodings, encoding)
                distances = face_recognition.face_distance(self.db.encodings, encoding)
                
                best_match_index = np.argmin(distances)
                if matches[best_match_index]:
                    name = self.db.names[best_match_index]
                    confidence = 1.0 - distances[best_match_index]
            
            results.append(RecognitionResult(name, confidence, location, encoding))
            
        return results

    def learn_face(self, name: str, frame: np.ndarray):
        """Extract face from frame and add to database."""
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        boxes = face_recognition.face_locations(rgb_frame)
        if not boxes:
            logger.warning("No face found to learn")
            return False
            
        encodings = face_recognition.face_encodings(rgb_frame, boxes)
        if encodings:
            self.db.add_face(name, encodings[0])
            logger.info(f"Learned new face: {name}")
            return True
        return False