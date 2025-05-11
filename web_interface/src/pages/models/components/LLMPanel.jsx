import React, { useState } from "react";

import { useAPI } from "../../../contexts/APIContext";
import { useToast } from "../../../contexts/ToastContext";
import { useLoadingScreen } from "../../../components/LoadingScreen";

const LLMPanel = ({ llmModelsList, setLlmModelsList }) => {
    const { llmModels, isResponseOk } = useAPI();
    const { withLoading } = useLoadingScreen();
    const { showToast } = useToast();

    const [apiKeyInputs, setApiKeyInputs] = useState({});

    const getCachedApiKey = (provider) => {
        return localStorage.getItem(`llm_api_key_${provider}`) || "";
    };

    const cacheApiKey = (provider, apiKey) => {
        localStorage.setItem(`llm_api_key_${provider}`, apiKey);
    };

    const onLoad = async (provider, model, needsApiKey) => {
        let apiKey = "";
        if (needsApiKey) {
            apiKey = apiKeyInputs[provider] ?? getCachedApiKey(provider);
            if (!apiKey) {
                showToast("Falta API key", `Debes introducir una API key para ${provider}`, "yellow");
                return;
            }
            cacheApiKey(provider, apiKey);
        }

        const response = await withLoading(() =>
            llmModels.load({
                provider,
                model,
                api_key: apiKey,
            })
        );

        if (isResponseOk(response)) {
            const { message, success } = response.data;
            if (success) {
                showToast("Modelo cargado", `${provider}/${model} cargado correctamente.`, "green");
                setLlmModelsList((list) =>
                    list.map((m) => (m.provider === provider && m.model === model ? { ...m, loaded: true } : m))
                );
            } else {
                showToast("Error al cargar modelo", message, "red");
            }
        } else {
            showToast("Error al cargar modelo", response.data.detail, "red");
        }
    };

    const onUnload = async (provider, model) => {
        const response = await withLoading(() =>
            llmModels.unload({
                provider,
                model,
            })
        );

        if (isResponseOk(response)) {
            const { message, success } = response.data;
            if (success) {
                showToast("Modelo liberado", `${provider}/${model} liberado correctamente.`, "green");
                setLlmModelsList((list) =>
                    list.map((m) => (m.provider === provider && m.model === model ? { ...m, loaded: false } : m))
                );
            } else {
                showToast("Error al liberar modelo", message, "red");
            }
        } else {
            showToast("Error al liberar modelo", response.data.detail, "red");
        }
    };

    const onActivate = async (provider, model) => {
        const response = await withLoading(() =>
            llmModels.activate({
                provider,
                model,
            })
        );

        if (isResponseOk(response)) {
            const { message, success } = response.data;
            if (success) {
                showToast("Modelo activado", `${provider}/${model} activado correctamente.`, "green");
                setLlmModelsList((list) =>
                    list.map((m) =>
                        m.provider === provider && m.model === model
                            ? { ...m, active: true }
                            : m.active
                            ? { ...m, active: false }
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

    return (
        <div className="list-group shadow-sm">
            {llmModelsList.map((m) => {
                const cache = getCachedApiKey(m.provider);
                const showInput = !m.loaded && m.needs_api_key;

                return (
                    <div
                        key={`${m.provider}/${m.model}`}
                        className={`list-group-item d-flex justify-content-between align-items-center ${
                            m.active ? "list-group-item-primary" : ""
                        }`}
                    >
                        <div className="me-3">
                            <h5 className="mb-1">
                                {m.provider} / <span className="text-capitalize">{m.model}</span>
                            </h5>
                            <div>
                                <span className={`badge me-2 ${m.loaded ? "bg-success" : "bg-secondary"}`}>
                                    {m.loaded ? "Cargado" : "No cargado"}
                                </span>
                                {m.active && <span className="badge bg-primary">Activo</span>}
                            </div>
                        </div>

                        <div className="btn-group align-items-center">
                            {showInput && (
                                <input
                                    type="password"
                                    placeholder="API Key"
                                    defaultValue={cache}
                                    onChange={(e) =>
                                        setApiKeyInputs((prev) => ({
                                            ...prev,
                                            [m.provider]: e.target.value,
                                        }))
                                    }
                                    className="form-control form-control-sm me-2"
                                    style={{ width: "200px" }}
                                />
                            )}

                            {!m.loaded && (
                                <button
                                    className="btn btn-sm btn-outline-success"
                                    onClick={() => onLoad(m.provider, m.model, m.needs_api_key)}
                                >
                                    Cargar
                                </button>
                            )}
                            {m.loaded && !m.active && (
                                <>
                                    <button
                                        className="btn btn-sm btn-outline-primary"
                                        onClick={() => onActivate(m.provider, m.model)}
                                    >
                                        Activar
                                    </button>
                                    <button
                                        className="btn btn-sm btn-outline-danger"
                                        onClick={() => onUnload(m.provider, m.model)}
                                    >
                                        Liberar
                                    </button>
                                </>
                            )}
                        </div>
                    </div>
                );
            })}
        </div>
    );
};

export default LLMPanel;
