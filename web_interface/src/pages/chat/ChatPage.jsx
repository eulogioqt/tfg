import React, { useState, useEffect, useRef } from "react";

import ChatSidebar from "./components/ChatSidebar";
import ChatFooter from "./components/ChatFooter";
import ChatMessageArea from "./components/ChatMessageArea";
import ChatHeader from "./components/ChatHeader";

import { useWebSocket } from "../../contexts/WebSocketContext";
import { useEventBus } from "../../contexts/EventBusContext";
import { useWindowSize, BREAKPOINTS } from "../../hooks/useWindowSize";
import { v4 as uuidv4 } from "uuid";

// Poner que cuando el chat no tiene mensajes el text area y demas este en medio
// Hacer un chatcontext o un collapsedcontext si se extiende todo demasiado y hay muchas cosas que ir pasando
// al menos collapsedcontext por el momento
const ChatPage = () => {
    const { sendMessage, isConnected } = useWebSocket();
    const { subscribe } = useEventBus();
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
            const id = uuidv4();
            const messageWithId = {
                type: "PROMPT",
                data: {
                    id: id,
                    value: inputMessage,
                },
            };

            sendMessage(messageWithId);
            addMessage(inputMessage, id, true);
        }
    };

    const handleNewChat = () => {
        setMessages([]);
        chatAreaRef.current?.clear();
        if (width < BREAKPOINTS.MD) setCollapsed(true);
    };

    useEffect(() => {
        const handlePromptResponse = (newResponse) => {
            const isResponseAdded = messages.filter((m) => !m.isHuman).some((m) => m.id === newResponse.id);
            if (!isResponseAdded) {
                addMessage(newResponse.value, newResponse.id, false);
            }
        };

        const unsubscribe = subscribe("PROMPT-RESPONSE", handlePromptResponse);
        return () => unsubscribe();
    }, []);

    return (
        <>
            <div className="d-flex vh-100">
                <ChatSidebar collapsed={collapsed} setCollapsed={setCollapsed} handleNewChat={handleNewChat} />

                <div className="d-flex flex-column mt-5 flex-grow-1" style={{ overflow: "hidden" }}>
                    <ChatHeader collapsed={collapsed} setCollapsed={setCollapsed} handleNewChat={handleNewChat} />

                    <ChatMessageArea messages={messages} />

                    <ChatFooter chatAreaRef={chatAreaRef} handleSend={handleSend} />
                </div>
            </div>
        </>
    );
};

export default ChatPage;
