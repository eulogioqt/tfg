import React, { useReducer, useEffect, useContext, createContext, useRef, useState } from "react";
import { v4 as uuidv4 } from "uuid";

const WebSocketContext = createContext();

const R2W_MESSAGE_TYPE = { // hacer objeto que wrapee el protocolo ros2web
    MESSAGE: "MESSAGE",
    TOPIC: "TOPIC"
}

const MESSAGE_TYPE = {
    RESPONSE: "RESPONSE",
    INIT: "INIT",
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
            const ws = new WebSocket("ws://" + serverIP + ":8765");

            console.log("Intentando conectar con el servidor WebSocket");

            ws.onopen = () => {
                console.log("Conexión establecida con el servidor WebSocket");

                setConnected(true);
                socketRef.current = ws;
            };

            ws.onmessage = (event) => {
                const message = JSON.parse(event.data);

                if (message.type === R2W_MESSAGE_TYPE.MESSAGE) { // High level protocol
                    const lowMessage = JSON.parse(message.data);

                    const type = lowMessage.type;
                    const data = lowMessage.data;

                    if (type === MESSAGE_TYPE.RESPONSE) {
                        console.log("Respuesta recibida:", message);
                        setPromptResponse(data);
                    } else if (type === MESSAGE_TYPE.INIT) {
                    } else {
                        console.log("Tipo de mensaje desconocido:", type, data);
                    }
                } else if (message.type === R2W_MESSAGE_TYPE.TOPIC) {
                    const lowMessage = message.data;

                    const topic = lowMessage.topic;
                    const name = lowMessage.name;
                    const value = lowMessage.value;

                    dispatch({ type: name, payload: value });
                } else {
                    console.log("Tipo de mensaje R2W desconocido:", type, data);
                }
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

    const sendMessage = (message) => {
        const id = uuidv4();
        const messageWithId = { // low level protocol
            type: "PROMPT",
            data: {
                id: id,
                value: message,
            },
        };

        const messageR2W = { // high level protocol
            type: "MESSAGE",
            data: messageWithId
        }

        console.log("Sending message to ROS:", messageR2W);
        socketRef.current.send(JSON.stringify(messageR2W));

        return id; // Poner que si falla algo devuelva undefined indicando ha fallado algo
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
