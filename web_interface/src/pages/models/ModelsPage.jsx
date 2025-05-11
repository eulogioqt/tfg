import React, { useEffect, useState } from "react";

import { useAPI } from "../../contexts/APIContext";
import { useToast } from "../../contexts/ToastContext";

import TTSPanel from "./components/TTSPanel";
import STTPanel from "./components/STTPanel";
import LLMPanel from "./components/LLMPanel";

const ModelsPage = () => {
    const { ttsModels, sttModels, llmModels, isResponseOk } = useAPI();
    const { showToast } = useToast();

    const [activeTab, setActiveTab] = useState("tts");
    const [ttsModelsList, setTtsModelsList] = useState([]);
    const [sttModelsList, setSttModelsList] = useState([]);
    const [llmModelsList, setLlmModelsList] = useState([]);

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

        const fetchSTT = async () => {
            const response = await sttModels.getAll();
            if (isResponseOk(response)) {
                setSttModelsList(response.data);
            } else {
                showToast("Error al obtener modelos STT", response.data.detail, "red");
            }
        };

        fetchSTT();

        const fetchLLM = async () => {
            const response = await llmModels.getAll();
            if (isResponseOk(response)) {
                setLlmModelsList(response.data);
            } else {
                showToast("Error al obtener modelos LLM", response.data.detail, "red");
            }
        };

        fetchLLM();
    }, []);

    const renderPanel = () => {
        switch (activeTab) {
            case "tts":
                return <TTSPanel ttsModelsList={ttsModelsList} setTtsModelsList={setTtsModelsList} />;
            case "stt":
                return <STTPanel sttModelsList={sttModelsList} setSttModelsList={setSttModelsList} />;
            case "llm":
                return <LLMPanel llmModelsList={llmModelsList} setLlmModelsList={setLlmModelsList} />;
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
