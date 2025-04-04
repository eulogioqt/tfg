import React, { useReducer, useEffect, useContext, createContext, useRef, useState } from "react";
import R2WSocket from "../utils/R2WSocket";

const WebSocketContext = createContext();

const MESSAGE_TYPE = {
    RESPONSE: "RESPONSE",
};

const initialState = {};
const reducer = (state, action) => ({ ...state, [action.type]: action.payload });

export const WebSocketProvider = ({ children }) => {
    const [displayData, dispatch] = useReducer(reducer, initialState);
    const [promptResponse, setPromptResponse] = useState(undefined);
    const [isConnected, setConnected] = useState(false);

    const socketRef = useRef(null);

    useEffect(() => {
        const connectWebSocket = () => {
            const serverIP = window.location.hostname;
            const ws = new R2WSocket("ws://" + serverIP + ":8765"); // Intentar meter en el R2WSocket el retry tmb por defecto
            console.log("Intentando conectar con el servidor WebSocket");

            ws.onopen = () => {
                console.log("Conexión establecida con el servidor WebSocket");

                setConnected(true);
                socketRef.current = ws;
            };

            ws.onclose = () => {
                console.log("Conexión cerrada, intentando reconectar...");

                setConnected(false);
                setTimeout(connectWebSocket, 3000); // Intenta reconectar después de 3 segundos
            };

            ws.onerror = (error) => {
                console.error("Error en la conexión WebSocket:", error);

                setConnected(false);
                ws.close();
            };

            ws.ontopic = (event) => onTopic(event);
            ws.onmessage = (event) => onMessage(event);
        };

        if (!socketRef.current) connectWebSocket();

        return () => {
            if (socketRef.current) {
                console.log("Closing socket...");

                setConnected(false);
                socketRef.current.close();
            }
        };
    }, []);

    const onTopic = (event) => {
        const topic = event.topic;
        const name = event.name;
        const value = event.value;
        //console.log(event);
        //console.log(`[${topic}] ${value}`);
        if (name == "ACTUAL_PEOPLE")
            console.log(topic, name, value)
        dispatch({ type: name, payload: value });
    };

    const onMessage = (event) => {
        const type = event.type;
        const data = event.data;

        if (type === MESSAGE_TYPE.RESPONSE) {
            console.log("Respuesta recibida:", event);
            setPromptResponse(data); // hacer eso mejor, como minimo una lista appendeando las respuestas
        } else {
            console.log("Tipo de mensaje desconocido:", type, data);
        }
    };

    const sendMessage = (message) => {
        if (socketRef.current) socketRef.current.send(message);
        else console.log("No conectado al R2WSocket");
    };

    return (
        <WebSocketContext.Provider
            value={{
                displayData,
                promptResponse,
                isConnected,
                sendMessage,
            }}
        >
            {children}
        </WebSocketContext.Provider>
    );
};

export const useWebSocket = () => useContext(WebSocketContext);
