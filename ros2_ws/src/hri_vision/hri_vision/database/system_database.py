import sqlite3
from datetime import datetime
from enum import Enum
from typing import List

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
                person_name TEXT NOT NULL,
                origin TEXT NOT NULL
            )
        ''')

        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS sessions (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                person_name TEXT NOT NULL,
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
    def create_log(self, action, person_name, origin):
        timestamp = datetime.now().timestamp()
        self.cursor.execute('''
            INSERT INTO logs (timestamp, action, person_name, origin)
            VALUES (?, ?, ?, ?)
        ''', (timestamp, action, person_name, origin))
        self.conn.commit()

    def get_all_logs(self):
        self.cursor.execute('SELECT * FROM logs')
        rows = self.cursor.fetchall()
        return [dict(row) for row in rows]

    def get_log_by_name(self, person_name):
        self.cursor.execute('SELECT * FROM logs WHERE person_name = ?', (person_name,))
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

    def get_session_by_name(self, person_name):
        self.cursor.execute('SELECT * FROM sessions WHERE person_name = ?', (person_name,))
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
            'person_name': str,
            'start_time': str,
            'end_time': str,
            'detections': [
                [timestamp: str, score_face: float, score_classification: float, face_image_base64: str or ""],
                ...
            ]
        }
        """
        self.cursor.execute('''
            INSERT INTO sessions (person_name, start_time, end_time)
            VALUES (?, ?, ?)
        ''', (
            session_dict['person_name'],
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

