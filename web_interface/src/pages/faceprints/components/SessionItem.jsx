import React from "react";

const SessionItem = ({ session }) => {
    const formatDateTime = (timestamp) => {
        const date = new Date(timestamp * 1000);
        return date.toLocaleDateString() + " " + date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    };

    const formatDuration = (start, end) => {
        const seconds = (end - start).toFixed(0);
        const minutes = Math.floor(seconds / 60);
        const remainingSeconds = seconds % 60;
        const hours = Math.floor(minutes / 60);
        const remainingMinutes = minutes % 60;

        if (hours > 0) return `${hours}h ${remainingMinutes}min ${remainingSeconds}s`;
        if (minutes > 0) return `${minutes}min ${remainingSeconds}s`;
        return `${seconds}s`;
    };

    return (
        <div className="card shadow-sm p-3 mb-3">
            <div className="d-flex flex-column flex-md-row justify-content-between align-items-md-center">
                <div>
                    <div className="mb-1">
                        <strong>Inicio:</strong> {formatDateTime(session.start_time)}
                    </div>
                    <div className="mb-1">
                        <strong>Fin:</strong> {formatDateTime(session.end_time)}
                    </div>
                    <div className="small text-muted">ID sesión: {session.id}</div>
                </div>

                <div className="text-md-end mt-3 mt-md-0">
                    <div className="mb-1">
                        <strong>Duración:</strong> {formatDuration(session.start_time, session.end_time)}
                    </div>
                    <div>
                        <strong>Detecciones:</strong> {session.detections.length}
                    </div>
                </div>
            </div>
        </div>
    );
};

export default SessionItem;
