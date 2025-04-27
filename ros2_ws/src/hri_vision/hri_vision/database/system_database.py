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

        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS sessions (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                faceprint_id INTEGER NOT NULL,
                start_time TEXT NOT NULL,
                end_time TEXT
            )
        ''')

        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS detections (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                session_id INTEGER NOT NULL,
                timestamp TEXT NOT NULL,
                score_face REAL,
                score_classification REAL,
                face_image_base64 TEXT,
                FOREIGN KEY(session_id) REFERENCES sessions(id)
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

    def get_logs_by_faceprint_id(self, faceprint_id):
        self.cursor.execute('SELECT * FROM logs WHERE faceprint_id = ?', (faceprint_id,))
        rows = self.cursor.fetchall()
        return [dict(row) for row in rows]

    # ----------------- SESSIONS -----------------
    def get_all_sessions(self):
        self.cursor.execute('SELECT * FROM sessions')
        sessions = self.cursor.fetchall()
        result = []
        for session in sessions:
            session_dict = dict(session)
            session_id = session_dict['id']
            detections = self.get_detections_by_session(session_id)
            session_dict['detections'] = detections
            result.append(session_dict)
        return result

    def get_sessions_by_faceprint_id(self, faceprint_id):
        self.cursor.execute('SELECT * FROM sessions WHERE faceprint_id = ?', (faceprint_id,))
        sessions = self.cursor.fetchall()
        result = []
        for session in sessions:
            session_dict = dict(session)
            session_id = session_dict['id']
            detections = self.get_detections_by_session(session_id)
            session_dict['detections'] = detections
            result.append(session_dict)
        return result

    # ----------------- DETECTIONS -----------------
    def get_detections_by_session(self, session_id):
        self.cursor.execute('SELECT timestamp, score_face, score_classification, face_image_base64 FROM detections WHERE session_id = ?', (session_id,))
        rows = self.cursor.fetchall()
        return [
            [
                row['timestamp'],
                row['score_face'],
                row['score_classification'],
                row['face_image_base64'] if row['face_image_base64'] is not None else ""
            ]
            for row in rows
        ]

    # ----------------- SAVE SESSION + DETECTIONS -----------------
    def create_session_with_detections(self, session_dict):
        """
        session_dict: {
            'faceprint_id': int,
            'start_time': str,
            'end_time': str,
            'detections': [
                [timestamp: str, score_face: float, score_classification: float, face_image_base64: str or ""],
                ...
            ]
        }
        """
        self.cursor.execute('''
            INSERT INTO sessions (faceprint_id, start_time, end_time)
            VALUES (?, ?, ?)
        ''', (
            session_dict['faceprint_id'],
            session_dict['start_time'],
            session_dict['end_time']
        ))
        session_id = self.cursor.lastrowid

        for detection in session_dict.get('detections', []):
            self.cursor.execute('''
                INSERT INTO detections (session_id, timestamp, score_face, score_classification, face_image_base64)
                VALUES (?, ?, ?, ?, ?)
            ''', (
                session_id,
                detection[0],  # timestamp
                detection[1],  # score_face
                detection[2],  # score_classification
                detection[3] if detection[3] != "" else None  # face_image_base64
            ))

        self.conn.commit()

