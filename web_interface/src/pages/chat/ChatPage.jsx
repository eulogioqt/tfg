import React, { useRef } from "react";

import ChatSidebar from "./components/ChatSidebar";
import ChatFooter from "./components/ChatFooter";
import ChatMessageArea from "./components/ChatMessageArea";
import ChatHeader from "./components/ChatHeader";

import { useWindowSize, BREAKPOINTS } from "../../hooks/useWindowSize";
import { useChat } from "../../contexts/ChatContext";

const ChatPage = () => {
    const { collapsed, setCollapsed, messages, clearMessages, handleAudio, handleUploadAudio, handleSend } = useChat();
    const { width } = useWindowSize();

    const chatAreaRef = useRef(null);

    const handleNewChat = () => {
        clearMessages();
        chatAreaRef.current?.clear();
        if (width < BREAKPOINTS.MD) setCollapsed(true);
    };

    return (
        <>
            <div className="d-flex vh-100">
                <ChatSidebar collapsed={collapsed} setCollapsed={setCollapsed} handleNewChat={handleNewChat} />

                <div className="d-flex flex-column flex-grow-1" style={{ overflow: "hidden", marginTop: "76px" }}>
                    <ChatHeader collapsed={collapsed} setCollapsed={setCollapsed} handleNewChat={handleNewChat} />

                    <ChatMessageArea messages={messages} />

                    <ChatFooter
                        chatAreaRef={chatAreaRef}
                        handleAudio={handleAudio}
                        handleUploadAudio={handleUploadAudio}
                        handleSend={handleSend}
                    />
                </div>
            </div>
        </>
    );
};

export default ChatPage;
