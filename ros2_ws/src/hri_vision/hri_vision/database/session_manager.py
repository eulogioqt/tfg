from datetime import datetime, timedelta
from typing import Optional
from .system_database import SystemDatabase


class SessionManager:
    def __init__(self, db: SystemDatabase, timeout_seconds: int = 5, time_between_detections: int = 1):
        self.db = db
        self.timeout_seconds = timeout_seconds
        self.time_between_detections = time_between_detections

        self.actual_people = {}
        self.active_sessions = {}  # person_name -> SessionData

    def process_detection(self, name: str, score_face: float, score_classification: float, face_image_base64: Optional[str] = None):
        now = datetime.now().timestamp()

        self.actual_people[name] = now

        if name not in self.active_sessions:
            self.active_sessions[name] = {
                'person_name': name,
                'start_time': now,
                'detections': []
            }

        session = self.active_sessions[name]

        last_detection_time = self._get_last_detection_time(session)
        if (now - last_detection_time) > self.time_between_detections:
            session['detections'].append([
                now,
                score_face,
                score_classification,
                face_image_base64 or ""
            ])

    def get_all_last_seen(self):
        actual_people_time = {}
        for key, value in self.actual_people.items():
            actual_people_time[key] = datetime.now().timestamp() - value
        return actual_people_time
    
    def get_last_seen(self, name):
        return datetime.now().timestamp() - self.actual_people.get(name, 0)

    def check_timeouts(self):
        now = datetime.now().timestamp()

        for name, session in self.active_sessions.items():
            last_detection_time = self._get_last_detection_time(session, now)
            if (now - last_detection_time) > self.timeout_seconds:
                self._close_session(name)
    
    def _close_session(self, name: str):
        session = self.active_sessions.pop(name, None)
        if session is None:
            return

        session_data = {
            'person_name': session['person_name'],
            'start_time': session['start_time'],
            'end_time': session['detections'][-1][0],
            'detections': session['detections']
        }

        self.db.create_session_with_detections(session_data)

    def _get_last_detection_time(session, default=0):
        return default if len(session['detections']) == 0 else session['detections'][-1][0]