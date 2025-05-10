import React from "react";
import ChatFullTextArea from "./ChatFullTextArea";

const ChatFooter = ({ chatAreaRef, handleAudio, handleUploadAudio, handleSend }) => {
    return (
        <div className="px-md-5 px-2 d-flex align-items-center">
            <ChatFullTextArea
                chatAreaRef={chatAreaRef}
                handleAudio={handleAudio}
                handleUploadAudio={handleUploadAudio}
                handleSend={handleSend}
            />
        </div>
    );
};

export default ChatFooter;
