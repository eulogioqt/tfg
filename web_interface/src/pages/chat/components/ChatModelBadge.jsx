import React from "react";

const ChatModelBadge = ({ model, type }) => {
    const icons = {
        tts: "bi-volume-up",
        stt: "bi-mic",
        llm: "bi-robot",
    };

    if (model === undefined) {
        return (
            <span className="badge bg-primary me-2">
                <i className={`bi ${icons[type]} me-1`}></i>Cargando...
            </span>
        );
    }

    if (model === null) {
        return (
            <span className="badge bg-danger me-2">
                <i className={`bi ${icons[type]} me-1`}></i>Desconectado
            </span>
        );
    }

    if (model === "") {
        return (
            <span className="badge bg-warning me-2">
                <i className={`bi ${icons[type]} me-1`}></i>No activo
            </span>
        );
    }

    return (
        <span className="badge bg-success me-2">
            <i className={`bi ${icons[type]} me-1`}></i>
            {type == "llm" ? model.provider : model.model}
        </span>
    );
};

export default ChatModelBadge;
