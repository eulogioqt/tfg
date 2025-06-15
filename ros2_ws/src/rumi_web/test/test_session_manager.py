import datetime
from unittest.mock import Mock, patch

from rumi_web.database.session_manager import SessionManager


# Hybrid: verify detections are aggregated based on time_between_detections

def test_process_detection_time_filter():
    db = Mock()
    times = iter([0, 0.5, 1.5])

    class FakeDatetime(datetime.datetime):
        @classmethod
        def now(cls):
            return datetime.datetime.fromtimestamp(next(times))

    manager = SessionManager(db, timeout_seconds=5, time_between_detections=1)

    with patch('rumi_web.database.session_manager.datetime', FakeDatetime):
        manager.process_detection('1', 0.8, 0.9)
        manager.process_detection('1', 0.8, 0.9)  # within 1s, ignored
        manager.process_detection('1', 0.8, 0.9)  # after 1s, stored

    session = manager.active_sessions['1']
    assert len(session['detections']) == 2
