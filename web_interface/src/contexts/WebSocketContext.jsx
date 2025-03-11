import React, { useReducer, useEffect, useContext, createContext, useRef, useState } from "react";

const WebSocketContext = createContext();

const MESSAGE_TYPE = {
    DISPLAY_DATA: "DISPLAY_DATA",
    RESPONSE: "RESPONSE",
    INIT: "INIT",
};

const initialState = {};
const reducer = (state, action) => ({ ...state, [action.type]: action.payload });

export const WebSocketProvider = ({ children }) => {
    const [displayData, dispatch] = useReducer(reducer, initialState);
    const [promptResponse, setPromptResponse] = useState(undefined);

    const socketRef = useRef(null);

    useEffect(() => {
        const connectWebSocket = () => {
            const serverIP = window.location.hostname;
            const ws = new WebSocket("ws://" + serverIP + ":8765");

            console.log("Intentando conectar con el servidor WebSocket");

            ws.onopen = () => {
                console.log("Conexión establecida con el servidor WebSocket");
                socketRef.current = ws;
            };

            ws.onmessage = (event) => {
                const message = JSON.parse(event.data);

                const type = message.type;
                const data = message.data;

                if (type === MESSAGE_TYPE.DISPLAY_DATA) {
                    dispatch({ type: data.type, payload: data.value });
                } else if (type === MESSAGE_TYPE.RESPONSE) {
                    setPromptResponse(data); // sendMessage podria asignar un ID al prompt y al responder devolver ese ID para trackear
                } else if (type === MESSAGE_TYPE.INIT) {
                } else {
                    console.log("Tipo de mensaje desconocido:", type, data);
                }
            };

            ws.onclose = () => {
                console.log("Conexión cerrada, intentando reconectar...");
                setTimeout(connectWebSocket, 3000); // Intenta reconectar después de 3 segundos
            };

            ws.onerror = (error) => {
                console.error("Error en la conexión WebSocket:", error);
                ws.close();
            };
        };

        if (!socketRef.current) connectWebSocket();

        return () => {
            if (socketRef.current) {
                console.log("Closing socket...");
                socketRef.current.close();
            }
        };
    }, []);

    const sendMessage = (message) => {
        // poner que devuelva si success o no
        console.log("Sending message to ROS:", message);
        socketRef.current.send(message);
    };

    return (
        <WebSocketContext.Provider
            value={{
                displayData,
                promptResponse,
                sendMessage,
            }}
        >
            {children}
        </WebSocketContext.Provider>
    );
};

export const useWebSocket = () => useContext(WebSocketContext);
