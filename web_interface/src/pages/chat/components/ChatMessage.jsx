import React from "react";
import { useChat } from "../../../contexts/ChatContext";

const ChatMessage = ({ message }) => {
    const { playAudio } = useChat();
    const isHuman = message.isHuman;
    const backgroundColor = isHuman ? "#E0E0E0" : "#FFFFFF";
    const shadowColor = isHuman ? "#C0C0C0" : "#DDDDDD";

    return (
        <div className={`d-flex ${isHuman ? "justify-content-end" : "justify-content-start"}`}>
            {/* Tema audio */}
            {message.audio && message.sampleRate && (
                <div className="d-flex flex-column align-items-center justify-content-center mb-3">
                    <i
                        onClick={() => playAudio(message.audio, message.sampleRate)}
                        className="d-flex justify-content-center align-items-center bi bi-play bg-dark text-white me-1 rounded-circle border"
                        style={{ width: "48px", height: "48px", cursor: "pointer" }}
                    ></i>
                </div>
            )}

            <div
                className="d-inline-block py-2 mb-3"
                style={{
                    backgroundColor,
                    border: `1px solid ${shadowColor}`,
                    borderRadius: "8px",
                    maxWidth: "70%",
                    padding: "4px 10px",
                    wordWrap: "break-word",
                    lineHeight: "1rem",
                }}
            >
                {/* Contenido del mensaje */}
                <div className="d-flex flex-column align-items-start justify-content-between">
                    <span className={`w-100 ${isHuman ? "text-end" : "text-start"}`}>{message.value}</span>
                </div>

                {/* Info humanos */}
                {isHuman && (message.intent || message.sttModel) && (
                    <div className="text-end w-100 mt-1">
                        {message.sttModel && (
                            <span className="pe-0 py-0 badge text-muted text-end ms-1">
                                <i className="bi bi-mic me-1"></i>
                                {message.sttModel}
                            </span>
                        )}
                        {message.intent && (
                            <span className="pe-0 py-0 badge text-muted text-end ms-1">
                                <i className="bi bi-robot me-1"></i>
                                {message.intent}
                            </span>
                        )}
                    </div>
                )}

                {/* Info bot */}
                {!isHuman && (message.provider || message.ttsModel) && (
                    <div className="mt-1">
                        {message.provider && (
                            <span className="ps-0 py-0 badge text-muted me-1">
                                <i className="bi bi-robot me-1"></i>
                                {message.provider}
                            </span>
                        )}
                        {message.ttsModel && (
                            <span className="ps-0 py-0 badge text-muted me-1">
                                <i className="bi bi-volume-up me-1"></i>
                                {message.ttsModel}
                            </span>
                        )}
                    </div>
                )}
            </div>
        </div>
    );
};

export default ChatMessage;
