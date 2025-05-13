import React from "react";

import { useModels } from "../../contexts/ModelsContext";

import TTSPanel from "./components/TTSPanel";
import STTPanel from "./components/STTPanel";
import LLMPanel from "./components/LLMPanel";

const ModelsPage = () => {
    const { activeTab, setActiveTab, fetchFunctions } = useModels();

    const renderPanel = () => {
        switch (activeTab) {
            case "tts":
                return <TTSPanel />;
            case "stt":
                return <STTPanel />;
            case "llm":
                return <LLMPanel />;
            default:
                return null;
        }
    };

    return (
        <div className="container pt-4" style={{ marginTop: "76px" }}>
            <h2 className="mb-4 fw-bold">Gestión de Modelos</h2>

            <div className="row">
                <div className="col-md-3">
                    <div className="list-group shadow-sm">
                        <button
                            className={`list-group-item list-group-item-action ${activeTab === "tts" ? "active" : ""}`}
                            onClick={() => setActiveTab("tts")}
                        >
                            <i className="bi bi-volume-up-fill-fill me-2" />
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
                    <div className="card border rounded shadow-sm mb-5">
                        <div className="card-header d-flex justify-content-between align-items-center bg-light fw-semibold">
                            <div>
                                <i className="bi bi-gear me-2" />
                                Modelos {activeTab.toUpperCase()}
                            </div>

                            <button className="btn btn-outline-secondary" onClick={() => fetchFunctions[activeTab]()}>
                                <i className="bi bi-arrow-clockwise me-2" /> Recargar
                            </button>
                        </div>
                        <div className="card-body">{renderPanel()}</div>
                    </div>
                </div>
            </div>
        </div>
    );
};

export default ModelsPage;
