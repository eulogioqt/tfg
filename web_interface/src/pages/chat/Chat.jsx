import React, { useState, useEffect, useRef } from "react";
import { useNavigate } from "react-router-dom";

import Message from "./components/Message";
import { useWebSocket } from "../../contexts/WebSocketContext";

const Chat = () => {
    const { promptResponse, sendMessage } = useWebSocket();

    const [messages, setMessages] = useState([]);
    const [inputMessage, setInputMessage] = useState("");
    const [isReplying, setIsReplying] = useState(false);

    const navigate = useNavigate();
    const messagesEndRef = useRef(null); // Para el scroll smooth

    const addMessage = (text, isHuman) => {
        setMessages((oldMessages) => [...oldMessages, { text: text, isHuman: isHuman }]);
    };

    const handleSend = () => {
        if (inputMessage.length > 0) {
            sendMessage(inputMessage);
            addMessage(inputMessage, true);
            setInputMessage("");
        }
    };

    const handleKeyDown = (event) => {
        if (event.key === "Enter") {
            event.preventDefault();
            handleSend();
        }
    };

    useEffect(() => {
        const newResponse = promptResponse;
        if (newResponse !== undefined) addMessage(newResponse, false);
    }, [promptResponse]);

    useEffect(() => {
        if (messagesEndRef.current) messagesEndRef.current.scrollIntoView({ behavior: "smooth" });
    }, [messages]);

    return (
        <>
            <div className="d-flex flex-column vh-100">
                <div
                    className="w-100 d-flex align-items-center justify-content-between px-md-5 px-3"
                    style={{ minHeight: "75px", height: "75px", borderBottom: "1px solid #EAEAEA" }}
                >
                    <div className="d-flex align-items-center">
                        <img
                            className="img-fluid rounded-circle"
                            src="https://github.com/eulogioqt.png"
                            alt="User"
                            style={{ width: "50px", height: "50px" }}
                        />
                        <div className="ms-3 d-flex flex-column">
                            <span className="fw-bold">Eulogio</span>
                            <span>{isReplying ? "Escribiendo..." : "En línea"}</span>
                        </div>
                    </div>
                    <button className="btn btn-secondary" onClick={() => navigate("/")}>
                        Salir
                    </button>
                </div>
                <div
                    className="flex-grow-1 d-flex flex-column"
                    style={{ backgroundColor: "#F9EEE8", overflowY: "auto" }}
                >
                    <div className="d-flex flex-column flex-grow-1 px-md-5 px-3 py-3">
                        {messages.map((message, index) => (
                            <Message key={index} text={message.text} isHuman={message.isHuman} />
                        ))}
                        <div ref={messagesEndRef} />
                    </div>
                </div>
                <div
                    className="d-flex w-100 align-items-center"
                    style={{ minHeight: "75px", height: "75px", borderTop: "1px solid #EAEAEA" }}
                >
                    <div className="input-group px-md-5 px-3">
                        <input
                            type="text"
                            value={inputMessage}
                            className="form-control border border-0"
                            placeholder="Escribe un mensaje..."
                            onChange={(event) => setInputMessage(event.target.value)}
                            onKeyDown={handleKeyDown}
                        />
                        <button className="btn btn-outline-secondary" type="button" onClick={handleSend}>
                            Enviar
                        </button>
                    </div>
                </div>
            </div>
        </>
    );
};

export default Chat;
