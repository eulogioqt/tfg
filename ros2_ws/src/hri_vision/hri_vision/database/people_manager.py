"""TODO: Add module documentation."""
from datetime import datetime

from rumi_msgs.msg import SessionMessage


class PeopleManager:
"""TODO: Describe class."""
    def __init__(self, node):
    """TODO: Describe __init__.
Args:
    node (:obj:`Any`): TODO.
"""
        self.node = node
        self.actual_people = {}

    def process_detection(self, faceprint_id: int, score_face: float, score_classification: float):
    """TODO: Describe process_detection.
Args:
    faceprint_id (:obj:`Any`): TODO.
    score_face (:obj:`Any`): TODO.
    score_classification (:obj:`Any`): TODO.
"""
        self.actual_people[faceprint_id] = datetime.now().timestamp()
        self.node.publisher_session.publish(SessionMessage(
            faceprint_id=str(faceprint_id),
            detection_score=float(score_face),
            classification_score=float(score_classification)
        ))

    def get_all_last_seen(self):
    """TODO: Describe get_all_last_seen.
"""
        actual_people_time = {}
        for key, value in self.actual_people.items():
            actual_people_time[key] = datetime.now().timestamp() - value
        return actual_people_time
    
    def get_last_seen(self, faceprint_id):
    """TODO: Describe get_last_seen.
Args:
    faceprint_id (:obj:`Any`): TODO.
"""
        return datetime.now().timestamp() - self.actual_people.get(faceprint_id, 0)

    def _get_last_detection_time(self, session, default=0):
    """TODO: Describe _get_last_detection_time.
Args:
    session (:obj:`Any`): TODO.
    default (:obj:`Any`): TODO.
"""
        return default if len(session['detections']) == 0 else session['detections'][-1][0]
