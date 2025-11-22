"""
Gun.js Storage Manager for distributed encrypted storage.
Ported from alanchelmickjr/whoami for Johnny 5.
"""

import json
import time
import hashlib
import base64
import os
import sqlite3
import threading
import queue
import logging
import socket
import struct
import pickle
from typing import Dict, List, Optional, Any, Tuple, Set
from dataclasses import dataclass, field
from enum import Enum
from cryptography.fernet import Fernet

# Configure logging
logger = logging.getLogger(__name__)

class MemoryCategory(Enum):
    """Categories of memory with different sharing policies."""
    PRIVATE = "private"  # Robot's secrets, never shared
    FAMILY = "family"    # Shared with trusted siblings
    PUBLIC = "public"    # Shared with all robots

class TrustLevel(Enum):
    """Trust levels for peer robots."""
    NONE = 0
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    SIBLING = 4  # Full trust

@dataclass
class Memory:
    """Represents a stored memory."""
    id: str
    category: MemoryCategory
    data: Dict[str, Any]
    timestamp: float = field(default_factory=time.time)
    encrypted: bool = False
    owner_id: str = ""
    tags: List[str] = field(default_factory=list)
    access_count: int = 0
    last_accessed: float = field(default_factory=time.time)
    shared_with: Set[str] = field(default_factory=set)

@dataclass
class Peer:
    """Represents a peer robot."""
    id: str
    address: str
    port: int
    trust_level: TrustLevel
    last_seen: float = field(default_factory=time.time)
    shared_memories: int = 0
    public_key: Optional[str] = None

class GunStorageManager:
    """
    Distributed encrypted storage manager inspired by Gun.js.
    """
    
    def __init__(self, robot_id: str, config: Optional[Dict] = None):
        self.robot_id = robot_id
        self.config = config or {}
        
        # Storage paths
        self.storage_dir = self.config.get('storage_dir', f'./gun_storage/{robot_id}')
        os.makedirs(self.storage_dir, exist_ok=True)
        
        # Encryption setup
        self.master_key = self._get_or_create_master_key()
        self.fernet = Fernet(self.master_key)
        
        # Local storage (SQLite for persistence)
        self.db_path = os.path.join(self.storage_dir, 'memories.db')
        self._init_database()
        
        # In-memory cache
        self.memory_cache: Dict[str, Memory] = {}
        self.peer_cache: Dict[str, Peer] = {}
        
        # Peer management
        self.trusted_peers: Dict[str, Peer] = {}
        self.pending_sync: queue.Queue = queue.Queue()
        
        # Network settings
        self.listen_port = self.config.get('listen_port', 8765)
        self.broadcast_interval = self.config.get('broadcast_interval', 30)
        
        # Privacy settings
        self.auto_share_family = self.config.get('auto_share_family', True)
        self.auto_share_public = self.config.get('auto_share_public', True)
        self.encryption_required = self.config.get('encryption_required', True)
        
        # Start background services
        self.running = False
        self.sync_thread = None
        self.listen_thread = None
        
        logger.info(f"GunStorageManager initialized for robot {robot_id}")
    
    def _get_or_create_master_key(self) -> bytes:
        """Get or create master encryption key."""
        key_file = os.path.join(self.storage_dir, '.master.key')
        
        if os.path.exists(key_file):
            with open(key_file, 'rb') as f:
                return f.read()
        else:
            key = Fernet.generate_key()
            with open(key_file, 'wb') as f:
                f.write(key)
            os.chmod(key_file, 0o600)
            return key
    
    def _init_database(self):
        """Initialize SQLite database for persistent storage."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS memories (
                id TEXT PRIMARY KEY,
                category TEXT NOT NULL,
                data TEXT NOT NULL,
                timestamp REAL NOT NULL,
                encrypted INTEGER NOT NULL,
                owner_id TEXT NOT NULL,
                tags TEXT,
                access_count INTEGER DEFAULT 0,
                last_accessed REAL,
                shared_with TEXT
            )
        ''')
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS peers (
                id TEXT PRIMARY KEY,
                address TEXT NOT NULL,
                port INTEGER NOT NULL,
                trust_level INTEGER NOT NULL,
                last_seen REAL NOT NULL,
                shared_memories INTEGER DEFAULT 0,
                public_key TEXT
            )
        ''')
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS sync_log (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                peer_id TEXT NOT NULL,
                memory_id TEXT NOT NULL,
                action TEXT NOT NULL,
                timestamp REAL NOT NULL,
                success INTEGER NOT NULL
            )
        ''')
        
        conn.commit()
        conn.close()

    # ... (Crucial methods ported for basic functionality)
    
    def store_private_memory(self, data: Dict[str, Any], tags: Optional[List[str]] = None) -> str:
        memory_id = self._generate_memory_id(data)
        encrypted_data = self._encrypt_data(data)
        
        memory = Memory(
            id=memory_id,
            category=MemoryCategory.PRIVATE,
            data=encrypted_data,
            encrypted=True,
            owner_id=self.robot_id,
            tags=tags or []
        )
        self._save_memory_to_db(memory)
        self.memory_cache[memory_id] = memory
        return memory_id

    def retrieve_memory(self, memory_id: str) -> Optional[Dict[str, Any]]:
        if memory_id in self.memory_cache:
            memory = self.memory_cache[memory_id]
        else:
            memory = self._load_memory_from_db(memory_id)
            if memory:
                self.memory_cache[memory_id] = memory
        
        if not memory:
            return None
            
        if memory.encrypted:
            try:
                return self._decrypt_data(memory.data)
            except Exception as e:
                logger.error(f"Failed to decrypt memory {memory_id}: {e}")
                return None
        return memory.data

    def _generate_memory_id(self, data: Dict[str, Any]) -> str:
        content = json.dumps(data, sort_keys=True)
        timestamp = str(time.time())
        combined = f"{self.robot_id}_{content}_{timestamp}"
        return hashlib.sha256(combined.encode()).hexdigest()[:16]
    
    def _encrypt_data(self, data: Dict[str, Any]) -> Dict[str, Any]:
        json_data = json.dumps(data)
        encrypted = self.fernet.encrypt(json_data.encode())
        return {'encrypted': base64.b64encode(encrypted).decode()}
    
    def _decrypt_data(self, encrypted_data: Dict[str, Any]) -> Dict[str, Any]:
        if 'encrypted' not in encrypted_data:
            return encrypted_data
        encrypted = base64.b64decode(encrypted_data['encrypted'])
        decrypted = self.fernet.decrypt(encrypted)
        return json.loads(decrypted.decode())

    def _save_memory_to_db(self, memory: Memory):
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute('''
            INSERT OR REPLACE INTO memories 
            (id, category, data, timestamp, encrypted, owner_id, tags, 
             access_count, last_accessed, shared_with)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (
            memory.id, memory.category.value, json.dumps(memory.data),
            memory.timestamp, int(memory.encrypted), memory.owner_id,
            json.dumps(memory.tags), memory.access_count, memory.last_accessed,
            json.dumps(list(memory.shared_with))
        ))
        conn.commit()
        conn.close()

    def _load_memory_from_db(self, memory_id: str) -> Optional[Memory]:
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute('SELECT * FROM memories WHERE id = ?', (memory_id,))
        row = cursor.fetchone()
        conn.close()
        
        if not row: return None
        
        return Memory(
            id=row[0], category=MemoryCategory(row[1]), data=json.loads(row[2]),
            timestamp=row[3], encrypted=bool(row[4]), owner_id=row[5],
            tags=json.loads(row[6]) if row[6] else [],
            access_count=row[7], last_accessed=row[8],
            shared_with=set(json.loads(row[9])) if row[9] else set()
        )