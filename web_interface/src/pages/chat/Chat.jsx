import React, { useState, useEffect, useRef } from "react";

import ChatSidebar from "./components/ChatSidebar";
import ChatFooter from "./components/ChatFooter";
import ChatMessageArea from "./components/ChatMessageArea";
import ChatHeader from "./components/ChatHeader";

import { useWebSocket } from "../../contexts/WebSocketContext";
import { useWindowSize, BREAKPOINTS } from "../../hooks/useWindowSize";

// Poner que cuando el chat no tiene mensajes el text area y demas este en medio
// Hacer un chatcontext o un collapsedcontext si se extiende todo demasiado y hay muchas cosas que ir pasando
// al menos collapsedcontext por el momento
const Chat = () => {
    const { promptResponse, sendMessage, isConnected } = useWebSocket();
    const { width } = useWindowSize();

    const [messages, setMessages] = useState([]);
    const [isReplying, setIsReplying] = useState(false);
    const [collapsed, setCollapsed] = useState(width < BREAKPOINTS.MD);

    const chatAreaRef = useRef(null);

    const addMessage = (text, id, isHuman) => {
        setMessages((oldMessages) => [...oldMessages, { text, id, isHuman }]);
        setIsReplying(isHuman);
    };

    const handleSend = (inputMessage) => {
        if (inputMessage.length > 0) {
            const id = isConnected ? sendMessage(inputMessage) : 10; // REMOVE if
            addMessage(inputMessage, id, true);
        }
    };

    const handleNewChat = () => {
        setMessages([]);
        chatAreaRef.current?.clear();
        if (width < BREAKPOINTS.MD) setCollapsed(true);
    };

    useEffect(() => {
        const newResponse = promptResponse;
        if (newResponse !== undefined) {
            const isResponseAdded = messages
                .filter((m) => !m.isHuman)
                .map((m) => m.id)
                .includes(newResponse.id);

            if (!isResponseAdded) addMessage(newResponse.value, newResponse.id, false);
        }
    }, [promptResponse]);

    return (
        <>
            <div className="d-flex vh-100">
                <ChatSidebar collapsed={collapsed} setCollapsed={setCollapsed} handleNewChat={handleNewChat} />

                <div className="d-flex flex-column flex-grow-1">
                    <ChatHeader collapsed={collapsed} setCollapsed={setCollapsed} handleNewChat={handleNewChat} />

                    <ChatMessageArea messages={messages} />

                    <ChatFooter chatAreaRef={chatAreaRef} handleSend={handleSend} />
                </div>
            </div>
        </>
    );
};

export default Chat;
