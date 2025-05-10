import React, { useRef, useEffect } from "react";
import ChatMessage from "./ChatMessage";

import { useChat } from "../../../contexts/ChatContext";

const ChatMessageArea = ({ messages }) => {
    const { transcribing } = useChat();

    const messagesEndRef = useRef(null);
    const firstScrollRef = useRef(false);

    useEffect(() => {
        if (messagesEndRef.current) {
            messagesEndRef.current.scrollIntoView({ behavior: firstScrollRef.current ? "smooth" : "auto" });
            firstScrollRef.current = true;
        }
    }, [messages]);

    return (
        <div className="flex-grow-1 d-flex flex-column" style={{ backgroundColor: "white", overflow: "auto" }}>
            <div className="container px-md-5 px-4 d-flex flex-column flex-grow-1 py-3">
                {messages.map((message, index) => (
                    <ChatMessage key={index} message={message} />
                ))}

                <div className="justify-content-end" style={{ display: transcribing ? "flex" : "none" }}>
                    Transcribiendo...
                </div>

                <div ref={messagesEndRef} />
            </div>
        </div>
    );
};

export default ChatMessageArea;
