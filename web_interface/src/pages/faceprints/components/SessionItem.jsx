import React from "react";

const SessionItem = ({session}) => {
    return (
        <div className="d-flex w-100 justify-content-between">
            <span>ID: {session.id}</span>
            <span>FaceprintID: {session.faceprint_id}</span>
            <span>Start time: {session.start_time}</span>
            <span>End time: {session.end_time}</span>
            <span>Detecciones: {session.detections.length}</span>
        </div>
    )
}

export default SessionItem;