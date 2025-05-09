import { useContext, createContext, useState } from "react";
import { useWebSocket } from "./WebSocketContext";

import { v4 as uuidv4 } from "uuid";

const ChatContext = createContext();

export const ChatProvider = ({ children }) => {
    const { sendMessage, isConnected } = useWebSocket();

    const [messages, setMessages] = useState([]);
    const [isReplying, setIsReplying] = useState(false);

    const clearMessages = () => setMessages([]);
    const addMessage = (text, id, isHuman) => {
        setMessages((oldMessages) => [...oldMessages, { text, id, isHuman }]);
        setIsReplying(isHuman);
    };

    const handleAudio = () => {
        const input = document.createElement("input");
        input.type = "file";
        input.accept = "audio/*";

        input.onchange = async (event) => {
            const file = event.target.files[0];
            if (!file) return;

            const arrayBuffer = await file.arrayBuffer();

            const audioContext = new AudioContext();
            const audioBuffer = await audioContext.decodeAudioData(arrayBuffer);

            const raw = audioBuffer.getChannelData(0);
            const int16Data = new Int16Array(raw.length);
            for (let i = 0; i < raw.length; i++) {
                const s = Math.max(-1, Math.min(1, raw[i]));
                int16Data[i] = s < 0 ? s * 0x8000 : s * 0x7fff;
            }

            const id = uuidv4();
            const chunkSize = 48000;

            for (let i = 0; i < int16Data.length; i += chunkSize) {
                const chunk = Array.from(int16Data.slice(i, i + chunkSize));
                const final = i + chunkSize >= int16Data.length;
                const index = Math.floor(i / chunkSize);

                const message = {
                    type: "AUDIO_PROMPT_CHUNK",
                    data: {
                        id: id,
                        chunk_index: index,
                        final: final,
                        audio: chunk,
                        sample_rate: audioBuffer.sampleRate,
                    },
                };

                sendMessage(message);
            }
        };

        input.click();
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

    return (
        <ChatContext.Provider
            value={{
                messages,
                isReplying,

                clearMessages,
                addMessage,
                handleAudio,
                handleSend,
            }}
        >
            {children}
        </ChatContext.Provider>
    );
};

export const useChat = () => useContext(ChatContext);
