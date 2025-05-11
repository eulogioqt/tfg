import React, { useState } from "react";

import { useAPI } from "../../../contexts/APIContext";
import { useToast } from "../../../contexts/ToastContext";
import { useLoadingScreen } from "../../../components/LoadingScreen";

import ActivateModal from "./ActivateModal";

const TTSPanel = ({ ttsModelsList, setTtsModelsList }) => {
    const { ttsModels, isResponseOk } = useAPI();
    const { withLoading } = useLoadingScreen();
    const { showToast } = useToast();

    const [showSpeakerModal, setShowSpeakerModal] = useState(false);
    const [selectedModel, setSelectedModel] = useState(null);
    const [selectedSpeaker, setSelectedSpeaker] = useState("");

    const onLoad = async (model) => {
        const response = await withLoading(() =>
            ttsModels.load({
                model: model,
            })
        );

        if (isResponseOk(response)) {
            const { message, success } = response.data;
            if (success) {
                showToast("Modelo cargado", `El modelo '${model}' se ha cargado correctamente.`, "green");
                setTtsModelsList((list) => list.map((m) => (m.model === model ? { ...m, loaded: true } : m)));
            } else {
                showToast("Error al cargar modelo", message, "red");
            }
        } else {
            showToast("Error al cargar modelo", response.data.detail, "red");
        }
    };

    const onUnload = async (model) => {
        const response = await withLoading(() =>
            ttsModels.unload({
                model: model,
            })
        );

        if (isResponseOk(response)) {
            const { message, success } = response.data;
            if (success) {
                showToast("Modelo liberado", `El modelo '${model}' se ha liberado correctamente.`, "green");
                setTtsModelsList((list) => list.map((m) => (m.model === model ? { ...m, loaded: false } : m)));
            } else {
                showToast("Error al liberar modelo", message, "red");
            }
        } else {
            showToast("Error al liberar modelo", response.data.detail, "red");
        }
    };

    const onActivate = async (model, speaker) => {
        const response = await withLoading(() =>
            ttsModels.activate({
                model: model,
                speaker: speaker,
            })
        );

        if (isResponseOk(response)) {
            const { message, success } = response.data;
            if (success) {
                showToast("Modelo activado", `El modelo '${model}' se ha activado correctamente.`, "green");
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
                showToast("Error al activar modelo", message, "red");
            }
        } else {
            showToast("Error al activar modelo", response.data.detail, "red");
        }
    };

    const openSpeakerModal = (model) => {
        if (model.speakers.length === 0) {
            onActivate(model.model, "");
        } else {
            setSelectedModel(model);
            setSelectedSpeaker(model.speaker || model.speakers[0] || "");
            setShowSpeakerModal(true);
        }
    };

    return (
        <>
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

            <div className="list-group shadow-sm">
                {ttsModelsList.map((model) => (
                    <div
                        key={model.model}
                        className={`list-group-item d-flex justify-content-between align-items-center ${
                            model.active ? "list-group-item-primary" : ""
                        }`}
                    >
                        <div className="me-3">
                            <h5 className="mb-1 text-capitalize">{model.model}</h5>
                            <p className="mb-1 text-muted small">
                                {model.speakers.length > 0
                                    ? `Voces: ${model.speakers.join(", ")}`
                                    : "Este modelo no tiene voces configurables"}
                            </p>
                            <div>
                                <span className={`badge me-2 ${model.loaded ? "bg-success" : "bg-secondary"}`}>
                                    {model.loaded ? "Cargado" : "No cargado"}
                                </span>
                                {model.active && (
                                    <span className="badge bg-primary">
                                        Activo {model.speaker ? `(${model.speaker})` : ""}
                                    </span>
                                )}
                            </div>
                        </div>

                        <div className="btn-group">
                            {!model.loaded && (
                                <button className="btn btn-sm btn-outline-success" onClick={() => onLoad(model.model)}>
                                    Cargar
                                </button>
                            )}
                            {model.loaded && !model.active && (
                                <>
                                    <button
                                        className="btn btn-sm btn-outline-primary"
                                        onClick={() => openSpeakerModal(model)}
                                    >
                                        Activar
                                    </button>
                                    <button
                                        className="btn btn-sm btn-outline-danger"
                                        onClick={() => onUnload(model.model)}
                                    >
                                        Liberar
                                    </button>
                                </>
                            )}
                        </div>
                    </div>
                ))}
            </div>
        </>
    );
};

export default TTSPanel;
