import React, { useEffect, useState } from "react";
import { useAudio } from "../../../contexts/AudioContext";
import { useChat } from "../../../contexts/ChatContext";

const ChatMessage = ({ message }) => {
    const { settings } = useChat();
    const { playAudio, stopCurrentAudio, activeAudioId } = useAudio();

    const [timer, setTimer] = useState(0);

    const isHuman = message.isHuman;
    const backgroundColor = isHuman ? "#E0E0E0" : "#FFFFFF";
    const shadowColor = isHuman ? "#C0C0C0" : "#DDDDDD";
    
    useEffect(() => {
        let intervalId;
        if (message.value != undefined) {
            if (timer > 0) setTimer(0);
        } else {
            intervalId = setInterval(() => {
                setTimer((prev) => prev + 100);
            }, 100);
        }

        return () => clearInterval(intervalId);
    }, [message.value]);

    const getMessageFooter = (isHuman, data, timestamp) => {
        const timeStr = new Date(timestamp).toLocaleTimeString([], {
            hour: "2-digit",
            minute: "2-digit",
            second: "2-digit",
            hour12: false,
        });

        return (
            <div className={"mt-1 d-flex justify-content-between " + (isHuman ? "" : "flex-row-reverse")}>
                <div>
                    <span className="px-0 py-0 badge text-muted">{timeStr}</span>
                </div>

                <div>
                    {data.map(
                        (item, index) =>
                            item.value && (
                                <span
                                    key={index}
                                    className={"px-0 py-0 badge text-muted " + (isHuman ? "ms-2" : "me-2")}
                                >
                                    <i className={`bi bi-${item.icon} me-1`}></i>
                                    {item.value}
                                </span>
                            )
                    )}
                </div>
            </div>
        );
    };

    return (
        <div className={`d-flex ${isHuman ? "justify-content-end" : "justify-content-end flex-row-reverse"}`}>
            {/* Boton reproducir audio ¿Pongo como whatsapp que sale las barritas de intensidad? */}
            {message.audio && message.sampleRate && (
                <div className="d-flex flex-column align-items-center justify-content-center mb-3">
                    <i
                        onClick={() => {
                            if (message.id + isHuman == activeAudioId) stopCurrentAudio();
                            else playAudio(message.audio, message.sampleRate, message.id + isHuman);
                        }}
                        title={message.id + isHuman == activeAudioId ? "Parar..." : "Reproducir..."}
                        className={`d-flex justify-content-center align-items-center text-white rounded-circle border ${
                            isHuman ? "me-2" : "ms-2"
                        } ${message.id + isHuman == activeAudioId ? "bg-danger bi-stop-fill" : "bg-dark bi-play-fill"}`}
                        style={{ fontSize: "1.5em", width: "48px", height: "48px", cursor: "pointer" }}
                    ></i>
                </div>
            )}

            <div
                className="d-inline-block py-2 mb-3"
                style={{
                    backgroundColor,
                    border: `1px solid ${shadowColor}`,
                    borderRadius: "8px",
                    maxWidth: "70%",
                    padding: "4px 10px",
                    wordWrap: "break-word",
                    lineHeight: "1.25em",
                }}
            >
                {/* Contenido del mensaje */}
                <div className="d-flex flex-column align-items-start justify-content-between text-justify">
                    <span className="w-100 text-start">
                        {message.value != undefined ? message.value : "Transcribiendo... (" + (timer / 1000).toFixed(1) + "s)"}
                    </span>
                </div>

                {/* Info humanos */}
                {isHuman &&
                    settings.showTechInfo &&
                    getMessageFooter(
                        true,
                        [
                            { value: message.sttModel, icon: "mic" },
                            { value: message.intent, icon: "compass" },
                        ],
                        message.timestamp
                    )}

                {/* Info bot */}
                {!isHuman &&
                    settings.showTechInfo &&
                    getMessageFooter(
                        false,
                        [
                            { value: message.provider, icon: "robot" },
                            { value: message.ttsModel, icon: "volume-up-fill" },
                        ],
                        message.timestamp
                    )}
            </div>
        </div>
    );
};

export default ChatMessage;
