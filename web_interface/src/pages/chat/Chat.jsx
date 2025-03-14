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
    const textAreaRef = useRef(null);

    const addMessage = (text, id, isHuman) => {
        setMessages((oldMessages) => [...oldMessages, { text: text, id: id, isHuman: isHuman }]);
        setIsReplying(isHuman);
    };

    const handleSend = () => {
        if (inputMessage.length > 0) {
            const id = sendMessage(inputMessage);
            addMessage(inputMessage, id, true);
            setInputMessage("");
            if (textAreaRef.current) {
                textAreaRef.current.style.height = "40px";
            }
        }
    };

    const handleKeyDown = (event) => {
        if (event.key === "Enter") {
            event.preventDefault();
            handleSend();
        }
    };

    const handleInputChange = (event) => {
        setInputMessage(event.target.value);
        const textarea = textAreaRef.current;
        if (textarea) {
            textarea.style.height = "40px";
            textarea.style.height = `${textarea.scrollHeight}px`;
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

                    {/* Bottom part*/}
                    <div className="d-flex w-100 align-items-center">
                        {/* Text area */}
                        <div className="container shadow-sm d-flex flex-column px-md-2 pb-2 py-1 border border-2 border-light-subtle rounded-5 mb-4">
                            <div className="input-group border-body-secondary mb-4">
                                <textarea
                                    ref={textAreaRef}
                                    value={inputMessage}
                                    className="small-scrollbar form-control ms-3 me-5"
                                    placeholder="Escribe un mensaje..."
                                    onChange={(event) => handleInputChange(event)}
                                    onKeyDown={handleKeyDown}
                                    rows="2"
                                    style={{
                                        resize: "none",
                                        overflowY: "auto",
                                        height: "40px",
                                        minHeight: "40px",
                                        maxHeight: "150px",
                                        border: "none",
                                        outline: "none",
                                    }}
                                />
                            </div>

                            <div className="d-flex justify-content-end align-items-end pe-1 pb-1">
                                <button className="btn btn-dark rounded-circle" type="button" onClick={handleSend}>
                                    <i className="bi bi-send"></i>
                                </button>
                            </div>
                        </div>
                    </div>
                </div>

                <style>{`
                    textarea:focus {
                        box-shadow: none !important;
                    }
                `}</style>
            </div>
        </>
    );
};

export default Chat;
