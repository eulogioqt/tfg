import React, { useRef, useEffect, useState } from "react";
import ChatMessage from "./ChatMessage";

import { useChat } from "../../../contexts/ChatContext";
import ChatMessageModal from "./ChatMessageModal";

const ChatMessageArea = () => {
    const { messages } = useChat();

    const [selectedMessage, setSelectedMessage] = useState(undefined);

    const messagesEndRef = useRef(null);
    const firstScrollRef = useRef(false);

    useEffect(() => {
        if (messagesEndRef.current) {
            messagesEndRef.current.scrollIntoView({ behavior: firstScrollRef.current ? "smooth" : "auto" });
            firstScrollRef.current = true;
        }
    }, [messages]);

    return (
        <>
            <ChatMessageModal
                isOpen={selectedMessage !== undefined}
                handleClose={() => setSelectedMessage(undefined)}
                message={selectedMessage}
            />

            <div className="flex-grow-1 d-flex flex-column" style={{ backgroundColor: "white", overflow: "auto" }}>
                <div className="container px-md-5 px-4 d-flex flex-column flex-grow-1 py-3">
                    {messages.map((message, index) => (
                        <ChatMessage key={index} message={message} onSelect={() => setSelectedMessage(message)} />
                    ))}

                    <div ref={messagesEndRef} />
                </div>
            </div>
        </>
    );
};

export default ChatMessageArea;
