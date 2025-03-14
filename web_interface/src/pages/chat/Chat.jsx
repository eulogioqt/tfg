import React, { useState, useEffect, useRef } from "react";
import { useNavigate } from "react-router-dom";

import Message from "./components/Message";
import { useWebSocket } from "../../contexts/WebSocketContext";

import sanchoHead from "../../assets/images/sancho_head.jpg";
import Sidebar from "./components/Sidebar";

const Chat = () => {
    const { promptResponse, sendMessage } = useWebSocket();

    const [messages, setMessages] = useState([]);
    const [inputMessage, setInputMessage] = useState("");
    const [isReplying, setIsReplying] = useState(false);
    const [collapsed, setCollapsed] = useState(false);

    const navigate = useNavigate();
    const messagesEndRef = useRef(null);

    const addMessage = (text, id, isHuman) => {
        setMessages((oldMessages) => [...oldMessages, { text: text, id: id, isHuman: isHuman }]);
        setIsReplying(isHuman);
    };

    const handleSend = () => {
        if (inputMessage.length > 0) {
            const id = sendMessage(inputMessage);
            addMessage(inputMessage, id, true);
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
        if (newResponse !== undefined) {
            const isResponseAdded = messages
                .filter((m) => !m.isHuman)
                .map((m) => m.id)
                .includes(newResponse.id);

            if (!isResponseAdded) addMessage(newResponse.value, newResponse.id, false);
        }
    }, [promptResponse]);

    useEffect(() => {
        if (messagesEndRef.current) messagesEndRef.current.scrollIntoView({ behavior: "smooth" });
    }, [messages]);

    return (
        <>
            <div className="d-flex vh-100">
                <Sidebar collapseCallback={setCollapsed} />

                <div className="d-flex flex-column flex-grow-1">
                    <div
                        className="w-100 d-flex align-items-center justify-content-between px-md-5 px-3"
                        style={{ minHeight: "75px", height: "75px", borderBottom: "1px solid #EAEAEA" }}
                    >
                        <div className="d-flex align-items-center">
                            <img
                                className="img-fluid rounded-circle"
                                src={sanchoHead}
                                alt="Sancho"
                                style={{ width: "50px", height: "50px", display: collapsed ? "block" : "block" }}
                            />

                            <span
                                className="fw-bold fs-2 ms-3"
                                style={{ whiteSpace: "nowrap", display: collapsed ? "block" : "none" }}
                            >
                                Sancho
                            </span>
                        </div>
                    </div>

                    <div
                        className="flex-grow-1 d-flex flex-column"
                        style={{
                            backgroundColor: "white",
                            overflowY: "auto",
                        }}
                    >
                        <div className="container-sm d-flex flex-column flex-grow-1 px-md-5 px-3 py-3">
                            {messages.map((message, index) => (
                                <Message key={index} message={message} />
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
            </div>
        </>
    );
};

export default Chat;
