import sqlite3

from datetime import datetime
from enum import Enum


class CONSTANTS:
    class LEVEL(str, Enum):
        INFO = "INFO"
        WARNING = "WARNING"
        ERROR = "ERROR"
        DEBUG = "DEBUG"

    class ORIGIN(str, Enum):
        ROS = "ROS"
        WEB = "WEB"

    class ACTION(str, Enum):
        ADD_CLASS = "add_class"
        RENAME_CLASS = "rename_class"
        DELETE_CLASS = "delete_class"
        ADD_FEATURES = "add_features"
        UPDATE_FACE = "update_face"

class SystemDatabase:
    def __init__(self, db_path='system.db'):
        self.conn = sqlite3.connect(db_path)
        self.conn.row_factory = sqlite3.Row
        self.cursor = self.conn.cursor()
        self._create_tables()

    def _create_tables(self):
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                level TEXT NOT NULL,
                origin TEXT NOT NULL,
                actor TEXT,
                action TEXT NOT NULL,
                target TEXT,
                message TEXT,
                metadata TEXT
            )
        ''')

        self.cursor.execute('CREATE INDEX IF NOT EXISTS idx_logs_timestamp ON logs (timestamp)')
        self.cursor.execute('CREATE INDEX IF NOT EXISTS idx_logs_action ON logs (action)')
        self.cursor.execute('CREATE INDEX IF NOT EXISTS idx_logs_origin ON logs (origin)')

        self.conn.commit()

    # ----------------- LOGS -----------------
    def create_log(self, level, origin, action, actor=None, target=None, message=None, metadata_json=None):
        if level not in CONSTANTS.LEVEL._value2member_map_:
            raise ValueError(f"Invalid level: {level}")
        if origin not in CONSTANTS.ORIGIN._value2member_map_:
            raise ValueError(f"Invalid origin: {origin}")
        if action not in CONSTANTS.ACTION._value2member_map_:
            raise ValueError(f"Invalid action: {action}")

        timestamp = datetime.now().timestamp()
        self.cursor.execute('''
            INSERT INTO logs (timestamp, level, origin, actor, action, target, message, metadata)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        ''', (timestamp, level, origin, actor, action, target, message, metadata_json))
        self.conn.commit()

    def get_all_logs(self):
        self.cursor.execute('SELECT * FROM logs')
        rows = self.cursor.fetchall()
        return [dict(row) for row in rows]

    def get_log_by_id(self, id):
        self.cursor.execute('SELECT * FROM logs WHERE id = ?', (id,))
        row = self.cursor.fetchone()
        return dict(row) if row else {}
