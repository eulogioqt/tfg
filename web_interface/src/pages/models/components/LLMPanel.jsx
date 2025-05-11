import React, { useState } from "react";
import { useAPI } from "../../../contexts/APIContext";
import { useToast } from "../../../contexts/ToastContext";
import { useLoadingScreen } from "../../../components/LoadingScreen";

const LLMPanel = ({ llmProvidersList, setLlmProvidersList }) => {
    const { llmModels, isResponseOk } = useAPI();
    const { showToast } = useToast();
    const { withLoading } = useLoadingScreen();

    const [apiKeys, setApiKeys] = useState({});
    const [showApiInput, setShowApiInput] = useState({});

    const getCachedApiKey = (provider) => localStorage.getItem(`llm_api_key_${provider}`) || "";
    const cacheApiKey = (provider, key) => localStorage.setItem(`llm_api_key_${provider}`, key);

    const onStartLoad = (provider, executedLocally, needsApiKey, model = null) => {
        if (needsApiKey) {
            const cached = getCachedApiKey(provider);
            setApiKeys((prev) => ({ ...prev, [provider]: cached }));
            setShowApiInput((prev) => ({ ...prev, [provider]: model || true }));
        } else {
            onConfirmLoad(provider, executedLocally, "", model);
        }
    };

    const onConfirmLoad = async (provider, executedLocally, apiKey, model = null) => {
        const providerData = llmProvidersList.find((p) => p.provider === provider);
        const needsApiKey = providerData.needs_api_key;

        if (needsApiKey && (!apiKey || apiKey.trim() === "")) {
            showToast("API Key requerida", "Debes introducir una API key para este proveedor.", "yellow");
            return;
        }

        const response = await withLoading(() =>
            llmModels.load({
                provider,
                model: model || providerData.models[0].model,
                api_key: apiKey,
            })
        );

        if (isResponseOk(response)) {
            const { message, success } = response.data;
            if (success) {
                if (needsApiKey) cacheApiKey(provider, apiKey);

                showToast("Modelo cargado", `El modelo '${model || "todos"}' ha sido cargado correctamente.`, "green");

                setLlmProvidersList((list) =>
                    list.map((p) =>
                        p.provider === provider
                            ? {
                                  ...p,
                                  models: p.models.map((m) =>
                                      model ? (m.model === model ? { ...m, loaded: true } : m) : { ...m, loaded: true }
                                  ),
                              }
                            : p
                    )
                );
                setShowApiInput((prev) => ({ ...prev, [provider]: false }));
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
                setLlmProvidersList((list) =>
                    list.map((p) =>
                        p.provider === provider
                            ? {
                                  ...p,
                                  models: p.models.map((m) => (m.model === model ? { ...m, loaded: false } : m)),
                              }
                            : p
                    )
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
                setLlmProvidersList((list) =>
                    list.map((p) =>
                        p.provider === provider
                            ? {
                                  ...p,
                                  models: p.models.map((m) => ({
                                      ...m,
                                      active: m.model === model,
                                  })),
                              }
                            : {
                                  ...p,
                                  models: p.models.map((m) => ({ ...m, active: false })),
                              }
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
            {llmProvidersList.map((providerData) => (
                <div key={providerData.provider} className="card my-3">
                    <div className="card-header d-flex justify-content-between align-items-center">
                        <h5 className="mb-0 text-capitalize">{providerData.provider}</h5>
                        {!providerData.executed_locally && !providerData.models[0].loaded && (
                            <div className="d-flex align-items-center">
                                {showApiInput[providerData.provider] ? (
                                    <>
                                        <input
                                            type="password"
                                            className="form-control form-control-sm me-2"
                                            placeholder="API Key"
                                            value={apiKeys[providerData.provider] || ""}
                                            onChange={(e) =>
                                                setApiKeys((prev) => ({
                                                    ...prev,
                                                    [providerData.provider]: e.target.value,
                                                }))
                                            }
                                            style={{ maxWidth: "200px" }}
                                        />
                                        <button
                                            className="btn btn-sm btn-outline-success"
                                            onClick={() =>
                                                onConfirmLoad(
                                                    providerData.provider,
                                                    false,
                                                    apiKeys[providerData.provider]
                                                )
                                            }
                                        >
                                            Confirmar
                                        </button>
                                    </>
                                ) : (
                                    <button
                                        className="btn btn-sm btn-outline-success"
                                        onClick={() =>
                                            onStartLoad(providerData.provider, false, providerData.needs_api_key)
                                        }
                                    >
                                        Cargar
                                    </button>
                                )}
                            </div>
                        )}
                    </div>
                    <div className="card-body p-2">
                        {providerData.models.map((model) => (
                            <div
                                key={model.model}
                                className={`list-group-item d-flex justify-content-between align-items-center ${
                                    model.active ? "list-group-item-primary" : ""
                                }`}
                            >
                                <div className="me-3">
                                    <h6 className="mb-1 text-capitalize">{model.model}</h6>
                                    <div>
                                        <span className={`badge me-2 ${model.loaded ? "bg-success" : "bg-secondary"}`}>
                                            {model.loaded ? "Cargado" : "No cargado"}
                                        </span>
                                        {model.active && <span className="badge bg-primary">Activo</span>}
                                    </div>
                                </div>

                                <div className="btn-group align-items-center">
                                    {providerData.executed_locally && !model.loaded && (
                                        <div className="btn-group align-items-center">
                                            {showApiInput[providerData.provider] == model.model ? (
                                                <>
                                                    <input
                                                        type="password"
                                                        className="form-control form-control-sm me-2"
                                                        placeholder="API Key"
                                                        value={apiKeys[providerData.provider] || ""}
                                                        onChange={(e) =>
                                                            setApiKeys((prev) => ({
                                                                ...prev,
                                                                [providerData.provider]: e.target.value,
                                                            }))
                                                        }
                                                        style={{ maxWidth: "200px" }}
                                                    />
                                                    <button
                                                        className="btn btn-sm btn-outline-success"
                                                        onClick={() =>
                                                            onConfirmLoad(
                                                                providerData.provider,
                                                                false,
                                                                apiKeys[providerData.provider],
                                                                model.model
                                                            )
                                                        }
                                                    >
                                                        Confirmar
                                                    </button>
                                                </>
                                            ) : (
                                                <button
                                                    className="btn btn-sm btn-outline-success"
                                                    onClick={() =>
                                                        providerData.needs_api_key
                                                            ? onStartLoad(
                                                                  providerData.provider,
                                                                  true,
                                                                  true,
                                                                  model.model
                                                              )
                                                            : onConfirmLoad(
                                                                  providerData.provider,
                                                                  true,
                                                                  "",
                                                                  model.model
                                                              )
                                                    }
                                                >
                                                    Cargar
                                                </button>
                                            )}
                                        </div>
                                    )}

                                    {providerData.executed_locally && model.loaded && !model.active && (
                                        <>
                                            <button
                                                className="btn btn-sm btn-outline-primary"
                                                onClick={() => onActivate(providerData.provider, model.model)}
                                            >
                                                Activar
                                            </button>
                                            <button
                                                className="btn btn-sm btn-outline-danger"
                                                onClick={() => onUnload(providerData.provider, model.model)}
                                            >
                                                Liberar
                                            </button>
                                        </>
                                    )}

                                    {!providerData.executed_locally && model.loaded && !model.active && (
                                        <button
                                            className="btn btn-sm btn-outline-primary"
                                            onClick={() => onActivate(providerData.provider, model.model)}
                                        >
                                            Activar
                                        </button>
                                    )}
                                </div>
                            </div>
                        ))}
                    </div>
                </div>
            ))}
        </div>
    );
};

export default LLMPanel;
