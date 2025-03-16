import React, { useState, useEffect, useRef } from "react";
import { useNavigate } from "react-router-dom";

import Message from "./components/Message";
import { useWebSocket } from "../../contexts/WebSocketContext";

import sanchoHead from "../../assets/images/sancho_head.jpg";
import Sidebar from "./components/Sidebar";
import { useWindowSize, BREAKPOINTS } from "../../hooks/useWindowSize";

// Refactorizar para que haya por un lado mensajes, por otro lado top part y por otro lado bottom part y aqui
// se mezcle todo, sidebar, top, bottom y mensajes part
// Poner que cuando el chat no tiene mensajes el text area y demas este en medio

// Hacer un componente solo para el textarea y que tenga un callback con useImperativeHandle
// y entonces que tenga un focus un clear un getvalue y vaya todo solo, y que tenga un callback para los envios

const Chat = () => {
    const { promptResponse, sendMessage } = useWebSocket();
    const { width } = useWindowSize();

    const [messages, setMessages] = useState([]);
    const [inputMessage, setInputMessage] = useState("");
    const [isReplying, setIsReplying] = useState(false);
    const [collapsed, setCollapsed] = useState(width < BREAKPOINTS.MD);

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
            clearInput();
        }
    };

    const handleKeyDown = (event) => {
        if (event.key === "Enter") {
            event.preventDefault();
            handleSend();
        }
    };

    const handleInputChange = (text) => {
        setInputMessage(text);
        const textarea = textAreaRef.current;
        if (textarea) {
            textarea.style.height = "40px";
            textarea.style.height = `${textarea.scrollHeight}px`;
        }
    };

    const handleNewChat = () => {
        setMessages([]);
        clearInput();
        if (width < BREAKPOINTS.MD) setCollapsed(true);
    };

    const clearInput = () => {
        setInputMessage("");
        const textarea = textAreaRef.current;
        if (textarea) textarea.style.height = "40px";
    };

    const focusTextArea = () => {
        const textarea = textAreaRef.current;
        if (textarea) {
            textarea.focus();
            textarea.setSelectionRange(textarea.value.length, textarea.value.length);
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
                <Sidebar collapsed={collapsed} setCollapsed={setCollapsed} handleNewChat={handleNewChat} />

                {/* Top part */}
                <div className="d-flex flex-column flex-grow-1">
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
                                <i class="bi bi-list"></i>
                            </button>

                            <div className="d-flex justify-content-center align-items-center">
                                <img
                                    className="img-fluid rounded-circle"
                                    src={sanchoHead}
                                    alt="Sancho"
                                    style={{
                                        width: "50px",
                                        height: "50px",
                                        display: collapsed || width < BREAKPOINTS.MD ? "block" : "block",
                                    }}
                                />

                                <span
                                    className="fw-bold fs-2 ms-3"
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

                    {/* Chat part */}
                    <div
                        className="flex-grow-1 d-flex flex-column"
                        style={{
                            backgroundColor: "white",
                            overflowY: "auto",
                        }}
                    >
                        <div className="container px-md-5 px-4 d-flex flex-column flex-grow-1 py-3">
                            {messages.map((message, index) => (
                                <Message key={index} message={message} />
                            ))}
                            <div ref={messagesEndRef} />
                        </div>
                    </div>

                    {/* Bottom part*/}
                    <div className="px-md-5 px-2 d-flex align-items-center">
                        {/* Text area */}
                        <div
                            className="container d-flex flex-column pb-2 py-1 mb-4 shadow-sm border border-2 border-light-subtle"
                            style={{ borderRadius: "24px", cursor: "text" }}
                            onClick={focusTextArea}
                        >
                            <div className="input-group border-body-secondary mb-2">
                                <textarea
                                    ref={textAreaRef}
                                    value={inputMessage}
                                    className="small-scrollbar form-control ms-0 ps-0 me-0 pe-0"
                                    placeholder="Escribe un mensaje..."
                                    onClick={(event) => event.stopPropagation()}
                                    onChange={(event) => handleInputChange(event.target.value)}
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

                            <div className="d-flex justify-content-end align-items-end pe-0 pb-0">
                                <button className="btn btn-black rounded-circle" type="button" onClick={handleSend}>
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
