import { useContext, createContext, useState, useEffect } from "react";
import { useWebSocket } from "./WebSocketContext";
import { useEventBus } from "./EventBusContext";

import { v4 as uuidv4 } from "uuid";

const ChatContext = createContext();

export const ChatProvider = ({ children }) => {
    const { sendMessage, isConnected } = useWebSocket();
    const { subscribe } = useEventBus();

    const [messages, setMessages] = useState([]);
    const [isReplying, setIsReplying] = useState(false);

    const clearMessages = () => setMessages([]);
    const addMessage = (text, id, isHuman) => {
        setMessages((oldMessages) => [...oldMessages, { text, id, isHuman }]);
        setIsReplying(isHuman);
    };

    useEffect(() => {
        const bufferRef = { current: [] };

        const processPT = (e) => addMessage(e.value, e.id, true);
        const processARC = (e) => {
            bufferRef.current.push(e.audio);
            if (e.final) {
                const finalAudio = bufferRef.current.flat();
                bufferRef.current = [];

                playAudio(finalAudio, e.sample_rate);
            }
        };

        const unsubscribePT = subscribe("ROS_MESSAGE_PROMPT_TRANSCRIPTION", processPT);
        const unsubscribeARC = subscribe("ROS_MESSAGE_AUDIO_RESPONSE_CHUNK", processARC);

        return () => {
            unsubscribePT();
            unsubscribeARC();
        };
    }, []);

    const playAudio = (audio, sampleRate) => {
        if (!audio || audio.length === 0) {
            console.warn("No audio data to play");
            return;
        }

        const context = new AudioContext({ sampleRate });
        const float32 = new Float32Array(audio.length);

        for (let i = 0; i < audio.length; i++) {
            float32[i] = Math.max(-1, Math.min(1, audio[i] / 32768));
        }

        const buffer = context.createBuffer(1, float32.length, sampleRate);
        buffer.copyToChannel(float32, 0);

        const source = context.createBufferSource();
        source.buffer = buffer;
        source.connect(context.destination);

        source.start();
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
