import React, { useState } from "react";
import { useAPI } from "../../../contexts/APIContext";
import { useToast } from "../../../contexts/ToastContext";
import ActivateModal from "./ActivateModal";

const TTSPanel = ({ ttsModelsList, setTtsModelsList }) => {
    const { ttsModels, isResponseOk } = useAPI();
    const { showToast } = useToast();

    const [showSpeakerModal, setShowSpeakerModal] = useState(false);
    const [selectedModel, setSelectedModel] = useState(null);
    const [selectedSpeaker, setSelectedSpeaker] = useState("");

    const onLoad = async (model) => {
        const response = await ttsModels.load({ model });
        if (isResponseOk(response)) {
            const { message, success } = response.data;
            if (success) {
                showToast("Modelo TTS cargado", `Se ha cargado el modelo TTS '${model}' correctamente.`, "green");
                setTtsModelsList((list) => list.map((m) => (m.model === model ? { ...m, loaded: true } : m)));
            } else {
                showToast("Error al cargar modelo TTS", message, "red");
            }
        } else {
            showToast("Error al cargar modelo TTS", response.data.detail, "red");
        }
    };

    const onUnload = async (model) => {
        const response = await ttsModels.unload({ model });
        if (isResponseOk(response)) {
            const { message, success } = response.data;
            if (success) {
                showToast("Modelo TTS liberado", `Se ha liberado el modelo TTS '${model}' correctamente.`, "green");
                setTtsModelsList((list) => list.map((m) => (m.model === model ? { ...m, loaded: false } : m)));
            } else {
                showToast("Error al liberar modelo TTS", message, "red");
            }
        } else {
            showToast("Error al liberar modelo TTS", response.data.detail, "red");
        }
    };

    const onActivate = async (model, speaker) => {
        const response = await ttsModels.activate({ model, speaker });
        if (isResponseOk(response)) {
            const { message, success } = response.data;
            if (success) {
                showToast("Modelo TTS activado", `Se ha activado el modelo TTS '${model}' correctamente.`, "green");
                setTtsModelsList((list) =>
                    list.map((m) =>
                        m.model === model
                            ? { ...m, active: true, speaker }
                            : m.active
                            ? { ...m, active: false, speaker: undefined }
                            : m
                    )
                );
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
                                <>
                                    <button
                                        className="btn btn-sm btn-outline-primary"
                                        onClick={() => openSpeakerModal(model)}
                                    >
                                        Activate
                                    </button>
                                    <button
                                        className="btn btn-sm btn-outline-danger"
                                        onClick={() => onUnload(model.model)}
                                    >
                                        Unload
                                    </button>
                                </>
                            )}
                        </div>
                    </div>
                ))}
            </div>

            <ActivateModal
                show={showSpeakerModal}
                model={selectedModel}
                selectedSpeaker={selectedSpeaker}
                setSelectedSpeaker={setSelectedSpeaker}
                onClose={() => setShowSpeakerModal(false)}
                onConfirm={() => {
                    if (selectedModel && selectedSpeaker) {
                        onActivate(selectedModel.model, selectedSpeaker);
                        setShowSpeakerModal(false);
                    }
                }}
            />
        </>
    );
};

export default TTSPanel;
