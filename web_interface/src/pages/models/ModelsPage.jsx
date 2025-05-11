import React, { useEffect, useState } from "react";

import { useAPI } from "../../contexts/APIContext";
import { useToast } from "../../contexts/ToastContext";

import TTSPanel from "./components/TTSPanel";
import STTPanel from "./components/STTPanel";
import LLMPanel from "./components/LLMPanel";

const ModelsPage = () => {
    const { ttsModels, isResponseOk } = useAPI();
    const { showToast } = useToast();

    const [activeTab, setActiveTab] = useState("tts");
    const [ttsModelsList, setTtsModelsList] = useState([]);

    useEffect(() => {
        const fetchTTS = async () => {
            const response = await ttsModels.getAll();
            if (isResponseOk(response)) {
                setTtsModelsList(response.data);
            } else {
                showToast("Error al obtener modelos TTS", response.data.detail, "red");
            }
        };

        fetchTTS();
    }, []);

    const renderPanel = () => {
        switch (activeTab) {
            case "tts":
                return <TTSPanel ttsModelsList={ttsModelsList} />;
            case "stt":
                return <STTPanel />;
            case "llm":
                return <LLMPanel />;
            default:
                return null;
        }
    };

    return (
        <div className="container pt-4 mt-5">
            <h2 className="mb-4 fw-bold">Gestión de Modelos</h2>

            <div className="row">
                <div className="col-md-3">
                    <div className="list-group shadow-sm">
                        <button
                            className={`list-group-item list-group-item-action ${activeTab === "tts" ? "active" : ""}`}
                            onClick={() => setActiveTab("tts")}
                        >
                            <i className="bi bi-volume-up-fill me-2" />
                            TTS
                        </button>
                        <button
                            className={`list-group-item list-group-item-action ${activeTab === "stt" ? "active" : ""}`}
                            onClick={() => setActiveTab("stt")}
                        >
                            <i className="bi bi-mic-fill me-2" />
                            STT
                        </button>
                        <button
                            className={`list-group-item list-group-item-action ${activeTab === "llm" ? "active" : ""}`}
                            onClick={() => setActiveTab("llm")}
                        >
                            <i className="bi bi-cpu-fill me-2" />
                            LLM
                        </button>
                    </div>
                </div>

                <div className="col-md-9">
                    <div className="card border rounded shadow-sm">
                        <div className="card-header bg-light fw-semibold">
                            <i className="bi bi-gear me-2" />
                            Modelos {activeTab.toUpperCase()}
                        </div>
                        <div className="card-body">{renderPanel()}</div>
                    </div>
                </div>
            </div>
        </div>
    );
};

export default ModelsPage;
