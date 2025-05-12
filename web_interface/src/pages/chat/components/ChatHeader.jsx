import React from "react";
import { useWindowSize, BREAKPOINTS } from "../../../hooks/useWindowSize";
import { useWebSocket } from "../../../contexts/WebSocketContext";

const ChatHeader = ({ collapsed, setCollapsed, handleNewChat }) => {
    const { width } = useWindowSize();
    const { isConnected } = useWebSocket();

    return (
        <div
            className="w-100 px-md-5 px-3 d-flex align-items-center justify-content-between"
            style={{ minHeight: "75px", height: "75px", borderBottom: "1px solid #EAEAEA" }}
        >
            <div className="d-flex align-items-center justify-content-between justify-content-md-start w-100">
                <button
                    className="btn btn-white me-3"
                    style={{ display: collapsed || width < BREAKPOINTS.MD ? "block" : "none" }}
                    onClick={() => setCollapsed((collapsed) => !collapsed)}
                >
                    <i className="bi bi-list"></i>
                </button>

                <div className="d-flex justify-content-center align-items-center">
                    <div
                        className={`rounded-circle border border-dark ${isConnected ? "bg-success" : "bg-danger"}`}
                        title={isConnected ? "Conectado" : "Desconectado"}
                        style={{
                            width: "16px",
                            height: "16px",
                            boxShadow: "0 0 5px rgba(0, 0, 0, 0.5)",
                            transition: "background-color 0.3s ease",
                        }}
                    />

                    <span
                        className="fw-bold fs-2 ms-3 my-0 py-0"
                        style={{
                            whiteSpace: "nowrap",
                            display: collapsed || width < BREAKPOINTS.MD ? "block" : "none",
                        }}
                    >
                        Sancho
                    </span>
                </div>

                <button
                    className="btn btn-white text-start"
                    style={{ display: width < BREAKPOINTS.MD ? "block" : "none" }}
                    onClick={() => handleNewChat()}
                >
                    <i className="bi bi-chat-dots"></i>
                </button>
            </div>
        </div>
    );
};

export default ChatHeader;
