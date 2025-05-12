import { useContext, createContext, useState, useEffect } from "react";
import { useWebSocket } from "./WebSocketContext";
import { useEventBus } from "./EventBusContext";
import { BREAKPOINTS, useWindowSize } from "../hooks/useWindowSize";

import { v4 as uuidv4 } from "uuid";
import NotConnectedModal from "../pages/chat/components/NotConnectedModal";

const ChatContext = createContext();

// ideas muy bomba: Toggle para revisar o no la transcripcion, para que se envie del tiron o que salga antes de enviar la trans
// toggle para el audio, si esta activo, cuando respnde sancho tmb se escucha la voz, si no no.
// Poner a parte del transcribiendo el generando respuesta el reproduciendo y eso, si eso incluso el tiempo que tarda y que lleva

export const ChatProvider = ({ children }) => {
    const { sendMessage, isConnected } = useWebSocket();
    const { subscribe } = useEventBus();
    const { width } = useWindowSize();

    const [messages, setMessages] = useState([]);
    const [collapsed, setCollapsed] = useState(width < BREAKPOINTS.MD);
    const [isOpenNCModal, setIsOpenNCModal] = useState(false);

    const clearMessages = () => setMessages([]);
    const addHumanMessage = (message) => {
        setMessages((prevMessages) => [...prevMessages, { ...message, timestamp: Date.now(), isHuman: true }]);
    };

    useEffect(() => {
        const processR = (e) => {
            setMessages((prevMessages) => {
                const updatedMessages = prevMessages.map((m) =>
                    m.isHuman && m.id === e.id ? { ...m, intent: e.intent } : m
                );

                return [...updatedMessages, { ...e, timestamp: Date.now(), isHuman: false }];
            });
        };
        const processPT = (e) => {
            setMessages((prevMessages) =>
                prevMessages.map((m) => (m.isHuman && m.id === e.id ? { ...m, sttModel: e.model, value: e.value } : m))
            );
        };
        const processAR = (e) => {
            setMessages((prevMessages) =>
                prevMessages.map((m) =>
                    !m.isHuman && m.id === e.id
                        ? { ...m, ttsModel: e.model, speaker: e.speaker, audio: e.audio, sampleRate: e.sample_rate }
                        : m
                )
            );

            playAudio(e.audio, e.sample_rate);
        };

        const unsubscribeR = subscribe("ROS_MESSAGE_RESPONSE", processR);
        const unsubscribePT = subscribe("ROS_MESSAGE_PROMPT_TRANSCRIPTION", processPT);
        const unsubscribeARC = subscribe("ROS_MESSAGE_AUDIO_RESPONSE", processAR);

        return () => {
            unsubscribeR();
            unsubscribePT();
            unsubscribeARC();
        };
    }, []);

    const handleUploadAudio = () => {
        if (!isConnected) return setIsOpenNCModal(true);

        const input = Object.assign(document.createElement("input"), {
            type: "file",
            accept: "audio/*",
            onchange: async (e) => {
                const file = e.target.files?.[0];
                if (file) await handleAudio(file);
            },
        });

        input.click();
    };

    const handleAudio = async (blob) => {
        if (!isConnected) return setIsOpenNCModal(true);

        const buffer = await blob.arrayBuffer();
        const audioCtx = new AudioContext();
        const audioBuffer = await audioCtx.decodeAudioData(buffer);

        const floatData = audioBuffer.getChannelData(0);
        const int16Data = Int16Array.from(floatData, (s) => Math.max(-1, Math.min(1, s)) * 0x7fff);

        const id = uuidv4();
        const audio = Array.from(int16Data);
        const sampleRate = audioBuffer.sampleRate;

        const message = {
            type: "AUDIO_PROMPT",
            data: { id, audio, sample_rate: sampleRate },
        };

        if (sendMessage(JSON.stringify(message))) {
            addHumanMessage({ id, audio, sampleRate });
        }
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

            if (sendMessage(JSON.stringify(messageWithId))) {
                addHumanMessage({ id: id, value: inputMessage }, true);
            }
        }
    };

    return (
        <ChatContext.Provider
            value={{
                collapsed,
                setCollapsed,

                messages,
                clearMessages,
                handleAudio,
                handleUploadAudio,
                handleSend,
            }}
        >
            <NotConnectedModal isOpen={isOpenNCModal} handleClose={() => setIsOpenNCModal(false)} />

            {children}
        </ChatContext.Provider>
    );
};

export const useChat = () => useContext(ChatContext);
