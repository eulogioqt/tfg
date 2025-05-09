import React from "react";
import ChatTextArea from "./ChatTextArea";

const ChatFullTextArea = ({ chatAreaRef, handleAudio, handleSend }) => {
    return (
        <div
            className="container d-flex flex-column pb-2 py-1 mb-4 shadow-sm border border-2 border-light-subtle"
            style={{ borderRadius: "24px", cursor: "text" }}
            onClick={() => chatAreaRef.current?.focus()}
        >
            <div className="input-group border-body-secondary mb-2">
                <ChatTextArea ref={chatAreaRef} onSend={handleSend} />
            </div>

            <div className="d-flex justify-content-end align-items-end pe-0 pb-0">
                <button className="btn btn-black rounded-circle me-1" type="button" onClick={() => handleAudio()}>
                    <i className="bi bi-mic"></i>
                </button>

                <button
                    className="btn btn-black rounded-circle"
                    type="button"
                    onClick={() => chatAreaRef.current?.send()}
                >
                    <i className="bi bi-send"></i>
                </button>
            </div>
        </div>
    );
};

export default ChatFullTextArea;
