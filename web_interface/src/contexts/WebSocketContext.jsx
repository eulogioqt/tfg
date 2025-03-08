import React, { useState } from "react";
import { useContext, createContext } from "react";

const WebSocketContext = createContext();

export const WebSocketProvider = ({ children }) => {
    return <WebSocketContext.Provider value={{}}>{children}</WebSocketContext.Provider>;
};

export const useWebSocket = () => useContext(WebSocketContext);
