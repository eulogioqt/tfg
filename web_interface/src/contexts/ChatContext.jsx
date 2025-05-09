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

            // Decodificar el audio a PCM usando Web Audio API
            const audioContext = new AudioContext();
            const audioBuffer = await audioContext.decodeAudioData(arrayBuffer);

            // Convertir a Int16
            const raw = audioBuffer.getChannelData(0); // Solo canal izquierdo (mono)
            const int16Data = new Int16Array(raw.length);

            for (let i = 0; i < raw.length; i++) {
                const s = Math.max(-1, Math.min(1, raw[i]));
                int16Data[i] = s < 0 ? s * 0x8000 : s * 0x7fff;
            }

            const id = uuidv4();
            const messageWithId = {
                type: "AUDIO_PROMPT",
                data: {
                    id: id,
                    audio: Array.from(int16Data), // Convertir a lista de int16
                    sample_rate: audioBuffer.sampleRate,
                },
            };
            console.log(JSON.stringify(messageWithId).length / 1024 / 1024 + " MB");
            console.log(messageWithId);

            sendMessage(messageWithId);
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
