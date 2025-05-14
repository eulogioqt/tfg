import sqlite3
from datetime import datetime
from enum import Enum


class CONSTANTS(str, Enum):
    ACTION_ADD_CLASS = "add_class"
    ACTION_RENAME_CLASS = "rename_class"
    ACTION_DELETE_CLASS = "delete_class"
    ACTION_ADD_FEATURES = "add_features"
    ACTION_UPDATE_FACE = "update_face"

    ORIGIN_ROS = "ROS"
    ORIGIN_WEB = "WEB"

class SystemDatabase:
    def __init__(self, db_path='system.db'):
        self.conn = sqlite3.connect(db_path)
        self.conn.row_factory = sqlite3.Row  # Esto permite acceder a los resultados como diccionarios
        self.cursor = self.conn.cursor()
        self._create_tables()

    def _create_tables(self):
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                action TEXT NOT NULL,
                faceprint_id INTEGER NOT NULL,
                origin TEXT NOT NULL
            )
        ''')

        self.conn.commit()

    # ----------------- LOGS -----------------
    def create_log(self, action, faceprint_id, origin = CONSTANTS.ORIGIN_ROS):
        timestamp = datetime.now().timestamp()
        self.cursor.execute('''
            INSERT INTO logs (timestamp, action, faceprint_id, origin)
            VALUES (?, ?, ?, ?)
        ''', (timestamp, action, faceprint_id, origin))
        self.conn.commit()

    def get_all_logs(self):
        self.cursor.execute('SELECT * FROM logs')
        rows = self.cursor.fetchall()
        return [dict(row) for row in rows]

    def get_log_by_id(self, id):
        self.cursor.execute('SELECT * FROM logs WHERE id = ?', (id,))
        row = self.cursor.fetchone()
        return dict(row) if row else {}

    def get_logs_by_faceprint_id(self, faceprint_id):
        self.cursor.execute('SELECT * FROM logs WHERE faceprint_id = ?', (faceprint_id,))
        rows = self.cursor.fetchall()
        return [dict(row) for row in rows]
