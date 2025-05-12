import { useContext, createContext, useState, useEffect } from "react";
import { useWebSocket } from "./WebSocketContext";
import { useEventBus } from "./EventBusContext";
import { BREAKPOINTS, useWindowSize } from "../hooks/useWindowSize";

import { v4 as uuidv4 } from "uuid";

const ChatContext = createContext();

// ideas muy bomba: Toggle para revisar o no la transcripcion, para que se envie del tiron o que salga antes de enviar la trans
// toggle para el audio, si esta activo, cuando respnde sancho tmb se escucha al voz, si no no.
// Mostrar el modelo de llm tts y stt que esta activo en el momento
// Poner a parte del transcribiendo el generando respuesta el reproduciendo y eso, si eso incluso el tiempo que tarda y que lleva
// Poner la intencion del texto clasificada
export const ChatProvider = ({ children }) => {
    const { sendMessage, isConnected } = useWebSocket();
    const { subscribe } = useEventBus();
    const { width } = useWindowSize();

    const [messages, setMessages] = useState([]);
    const [collapsed, setCollapsed] = useState(width < BREAKPOINTS.MD);
    const [transcribing, setTranscribing] = useState(false);
    const [isReplying, setIsReplying] = useState(false);

    const clearMessages = () => setMessages([]);
    const addMessage = (text, id, isHuman) => {
        setMessages((oldMessages) => [...oldMessages, { text, id, isHuman }]);
        setIsReplying(isHuman);
    };

    useEffect(() => {
        const processR = (e) => {
            console.log(e);
            const isResponseAdded = messages.filter((m) => !m.isHuman).some((m) => m.id === e.id);
            if (!isResponseAdded) {
                addMessage(e.value, e.id, false);
            }
        };
        const processPT = (e) => {
            setTranscribing(false);
            addMessage(e.value, e.id, true);
        };
        const processAR = (e) => playAudio(e.audio, e.sample_rate);

        const unsubscribeR = subscribe("ROS_MESSAGE_RESPONSE", processR);
        const unsubscribePT = subscribe("ROS_MESSAGE_PROMPT_TRANSCRIPTION", processPT);
        const unsubscribeARC = subscribe("ROS_MESSAGE_AUDIO_RESPONSE", processAR);

        return () => {
            unsubscribeR();
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

    const handleAudio = async (audioBlob) => {
        const arrayBuffer = await audioBlob.arrayBuffer();

        const audioContext = new AudioContext();
        const audioBuffer = await audioContext.decodeAudioData(arrayBuffer);

        const raw = audioBuffer.getChannelData(0); // Solo canal izquierdo (mono)
        const int16Data = new Int16Array(raw.length);
        for (let i = 0; i < raw.length; i++) {
            const s = Math.max(-1, Math.min(1, raw[i]));
            int16Data[i] = s < 0 ? s * 0x8000 : s * 0x7fff;
        }

        const audioPromptMessage = {
            type: "AUDIO_PROMPT",
            data: {
                id: crypto.randomUUID(),
                audio: Array.from(int16Data),
                sample_rate: audioBuffer.sampleRate,
            },
        };

        setTranscribing(true);
        sendMessage(JSON.stringify(audioPromptMessage));
    };

    const handleUploadAudio = () => {
        const input = document.createElement("input");
        input.type = "file";
        input.accept = "audio/*";

        input.onchange = async (event) => {
            const file = event.target.files[0];
            if (!file) return;

            await handleAudio(file);
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

            sendMessage(JSON.stringify(messageWithId));
            addMessage(inputMessage, id, true);
        }
    };

    return (
        <ChatContext.Provider
            value={{
                collapsed,
                setCollapsed,
                
                messages,
                isReplying,
                transcribing,

                clearMessages,
                addMessage,
                handleAudio,
                handleUploadAudio,
                handleSend,
            }}
        >
            {children}
        </ChatContext.Provider>
    );
};

export const useChat = () => useContext(ChatContext);
