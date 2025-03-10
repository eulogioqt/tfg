import { useEffect, useRef, useState } from "react";
import { useWebSocket } from "../../contexts/WebSocketContext";

const Chat = () => {
    const { displayData } = useWebSocket();

    const [count, setCount] = useState(0);
    const [data, setData] = useState([]);

    const messagesEndRef = useRef(null); // Referencia para el contenedor de mensajes

    useEffect(() => {
        const newValue = displayData["NUMBER"];
        if (newValue !== undefined) setData((prev) => [...prev, newValue]);
    }, [displayData["NUMBER"]]);

    useEffect(() => {
        if (messagesEndRef.current) messagesEndRef.current.scrollIntoView({ behavior: "smooth" });
    }, [data]);

    return (
        <div className="container d-flex flex-column justify-content-center align-items-center vh-100">
            <div
                className="flex-grow-1 d-flex flex-column w-100 mt-5"
                style={{ backgroundColor: "#F9EEE8", overflowY: "auto" }}
            >
                <div className="d-flex flex-column flex-grow-1 px-md-5 px-3 py-3">
                    {data.map((d) => (
                        <div className="d-flex justify-content-end">
                            <div
                                className="d-inline-block mb-2"
                                style={{
                                    backgroundColor: "white",
                                    borderRadius: "8px",
                                    maxWidth: "80%",
                                    padding: "8px",
                                }}
                            >
                                {d}
                            </div>
                        </div>
                    ))}
                    <div ref={messagesEndRef} />
                </div>
            </div>
            <div
                className="d-flex w-100 align-items-center"
                style={{ minHeight: "75px", height: "75px", borderTop: "1px solid #EAEAEA" }}
            >
                <div className="input-group px-md-5 px-3">
                    <input type="text" className="form-control border border-0" placeholder="Escribe un mensaje..." />
                    <button className="btn btn-outline-secondary" type="button">
                        Enviar
                    </button>
                </div>
            </div>
        </div>
    );
};

export default Chat;
