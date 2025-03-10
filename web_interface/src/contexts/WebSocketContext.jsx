import React, { useReducer, useEffect, useContext, createContext, useRef } from "react";

const WebSocketContext = createContext();

const initialState = {};
const reducer = (state, action) => ({ ...state, [action.type]: action.payload });

export const WebSocketProvider = ({ children }) => {
    const [displayData, dispatch] = useReducer(reducer, initialState);

    const socketRef = useRef(null);

    useEffect(() => {
        const connectWebSocket = () => {
            const serverIP = window.location.hostname;
            const ws = new WebSocket("ws://" + serverIP + ":8765");

            ws.onopen = () => {
                console.log("Conexión establecida con el servidor WebSocket");
                socketRef.current = ws;
            };

            ws.onmessage = (event) => {
                const message = JSON.parse(event.data);

                if (message.type === "DISPLAY_DATA") {
                    const data = message.data;
                    dispatch({ type: data.type, payload: data.value });
                } else if (message.type === "RESPONSE") {
                } else if (message.type === "INIT") {
                } else {
                    console.log("Mensaje desconocido:", message);
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

        if (!socketRef.current) {
            connectWebSocket();
        }

        return () => {
            if (socketRef.current) {
                console.log("Closing socket...");
                socketRef.current.close();
            }
        };
    }, []);

    const sendMessage = (message) => {
        console.log("Sending message to ROS: ", message);
        socketRef.current.send(message);
    };

    return (
        <WebSocketContext.Provider
            value={{
                displayData,
                sendMessage,
            }}
        >
            {children}
        </WebSocketContext.Provider>
    );
};

export const useWebSocket = () => useContext(WebSocketContext);
