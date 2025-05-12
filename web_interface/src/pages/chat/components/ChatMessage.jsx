import React from "react";

const ChatMessage = ({ message }) => {
    const isHuman = message.isHuman;
    const backgroundColor = isHuman ? "#E0E0E0" : "#FFFFFF";
    const shadowColor = isHuman ? "#C0C0C0" : "#DDDDDD";

    return (
        <div className={`d-flex ${isHuman ? "justify-content-end" : "justify-content-start"}`}>
            <div
                className="d-inline-block py-2 mb-3"
                style={{
                    backgroundColor,
                    border: `1px solid ${shadowColor}`,
                    borderRadius: "8px",
                    maxWidth: "70%",
                    padding: "4px 10px",
                    wordWrap: "break-word",
                    lineHeight: "1rem"
                }}
            >
                {/* Contenido del mensaje */}
                <div className="d-flex flex-column align-items-start justify-content-between">
                    <span className={`w-100 ${isHuman ? "text-end" : "text-start"}`}>{message.value}</span>
                </div>
            
                {/* Info humanos */}
                {isHuman && message.intent && (
                    <div className="mt-1">
                        <span 
                            className="p-0 badge text-muted text-end w-100 me-1" 
                            style={{ fontSize: "0.65rem" }}
                        >
                            <i className="bi bi-compass me-1"></i>{message.intent}
                        </span>
                    </div>
                )}

                {/* Info bot */}
                {!isHuman && (message.provider || message.model) && (
                    <div className="mt-1">
                        {message.provider && (
                            <span className="ps-0 py-0 badge text-muted me-1">
                                <i className="bi bi-robot me-1"></i>{message.provider}
                            </span>
                        )}
                        {message.model && (
                            <span className="px-0 py-0 badge text-muted">
                                <i className="bi bi-cpu me-1"></i>{message.model.split("/")[1] || message.model}
                            </span>
                        )}
                    </div>
                )}
            </div>
        </div>
    );
};

export default ChatMessage;
