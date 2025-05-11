import React, { useState } from "react";

const ModelsPage = () => {
    const [activeTab, setActiveTab] = useState("tts");

    const renderPanel = () => {
        switch (activeTab) {
            case "tts":
                return <div>Aquí van los modelos TTS</div>;
            case "stt":
                return <div>Aquí van los modelos STT</div>;
            case "llm":
                return <div>Aquí van los modelos LLM</div>;
            default:
                return null;
        }
    };

    return (
        <div className="container pt-3 mt-5">
            <h2 className="mb-4 fw-bold">Gestión de Modelos</h2>

            <div className="row">
                {/* Sidebar */}
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

                {/* Panel derecho */}
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
