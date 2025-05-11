import React, { useState } from "react";

import { useAPI } from "../../../contexts/APIContext";
import { useToast } from "../../../contexts/ToastContext";

const TTSPanel = ({ ttsModelsList }) => {
    const { ttsModels, isResponseOk } = useAPI();
    const { showToast } = useToast();

    const [showSpeakerModal, setShowSpeakerModal] = useState(false);
    const [selectedModel, setSelectedModel] = useState(null);
    const [selectedSpeaker, setSelectedSpeaker] = useState("");

    const onLoad = async (model) => {
        const response = await ttsModels.load({
            model: model,
        });

        if (isResponseOk(response)) {
            const { message, success } = response.data;
            if (success) {
                showToast("Modelo TTS cargado", `Se ha cargado el modelo TTS '${model}' correctamente.`, "green");
            } else {
                showToast("Error al cargar modelo TTS", message, "red");
            }
        } else {
            showToast("Error al cargar modelo TTS", response.data.detail, "red");
        }
    };

    const onUnload = async (model) => {
        const response = await ttsModels.unload({
            model: model,
        });

        if (isResponseOk(response)) {
            const { message, success } = response.data;
            if (success) {
                showToast("Modelo TTS descargado", `Se ha descargado el modelo TTS '${model}' correctamente.`, "green");
            } else {
                showToast("Error al descargar modelo TTS", message, "red");
            }
        } else {
            showToast("Error al descargar modelo TTS", response.data.detail, "red");
        }
    };

    const onActivate = async (model, speaker) => {
        const response = await ttsModels.activate({
            model: model,
            speaker: speaker,
        });

        if (isResponseOk(response)) {
            const { message, success } = response.data;
            if (success) {
                showToast("Modelo TTS activado", `Se ha activado el modelo TTS '${model}' correctamente.`, "green");
            } else {
                showToast("Error al activar modelo TTS", message, "red");
            }
        } else {
            showToast("Error al activar modelo TTS", response.data.detail, "red");
        }
    };

    const openSpeakerModal = (model) => {
        setSelectedModel(model);
        setSelectedSpeaker(model.speaker || "");
        setShowSpeakerModal(true);
    };

    const handleActivate = () => {
        if (selectedModel && selectedSpeaker) {
            onActivate(selectedModel.model, selectedSpeaker);
            setShowSpeakerModal(false);
        }
    };

    return (
        <>
            <div className="list-group">
                {ttsModelsList.map((model) => (
                    <div
                        key={model.model}
                        className={`list-group-item d-flex justify-content-between align-items-center ${
                            model.active ? "list-group-item-primary" : ""
                        }`}
                    >
                        <div>
                            <h5 className="mb-1 text-capitalize">{model.model}</h5>
                            <p className="mb-1 small">
                                {model.speakers.length > 0 ? `Speakers: ${model.speakers.join(", ")}` : "No speakers"}
                            </p>
                            <span className={`badge me-1 ${model.loaded ? "bg-success" : "bg-secondary"}`}>
                                {model.loaded ? "Loaded" : "Not loaded"}
                            </span>
                            {model.active && <span className="badge bg-info">Active ({model.speaker})</span>}
                        </div>

                        <div className="btn-group">
                            {!model.loaded && (
                                <button className="btn btn-sm btn-outline-success" onClick={() => onLoad(model.model)}>
                                    Load
                                </button>
                            )}
                            {model.loaded && !model.active && (
                                <button
                                    className="btn btn-sm btn-outline-primary"
                                    onClick={() => openSpeakerModal(model)}
                                >
                                    Activate
                                </button>
                            )}
                            {model.loaded && !model.active && (
                                <button className="btn btn-sm btn-outline-danger" onClick={() => onUnload(model.model)}>
                                    Unload
                                </button>
                            )}
                        </div>
                    </div>
                ))}
            </div>

            {/* Modal personalizado */}
            {showSpeakerModal && (
                <div className="modal show d-block" tabIndex="-1">
                    <div className="modal-dialog">
                        <div className="modal-content">
                            <div className="modal-header">
                                <h5 className="modal-title">Activate Model</h5>
                                <button
                                    type="button"
                                    className="btn-close"
                                    onClick={() => setShowSpeakerModal(false)}
                                ></button>
                            </div>
                            <div className="modal-body">
                                <label className="form-label">Select Speaker</label>
                                <select
                                    className="form-select"
                                    value={selectedSpeaker}
                                    onChange={(e) => setSelectedSpeaker(e.target.value)}
                                >
                                    <option value="">Select...</option>
                                    {selectedModel?.speakers.map((spk) => (
                                        <option key={spk} value={spk}>
                                            {spk}
                                        </option>
                                    ))}
                                </select>
                            </div>
                            <div className="modal-footer">
                                <button
                                    type="button"
                                    className="btn btn-secondary"
                                    onClick={() => setShowSpeakerModal(false)}
                                >
                                    Cancel
                                </button>
                                <button
                                    type="button"
                                    className="btn btn-primary"
                                    disabled={!selectedSpeaker}
                                    onClick={handleActivate}
                                >
                                    Activate
                                </button>
                            </div>
                        </div>
                    </div>
                    <div className="modal-backdrop fade show"></div>
                </div>
            )}
        </>
    );
};

export default TTSPanel;
