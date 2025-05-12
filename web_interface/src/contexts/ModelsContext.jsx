import React, { createContext, useContext, useEffect, useState } from "react";
import { useToast } from "./ToastContext";
import { useAPI } from "./APIContext";

const ModelsContext = createContext();

export const ModelsProvider = ({ children }) => {
    const { ttsModels, sttModels, llmModels, isResponseOk } = useAPI();
    const { showToast } = useToast();

    const [activeTab, setActiveTab] = useState("tts");
    const [ttsModelsList, setTtsModelsList] = useState(undefined);
    const [sttModelsList, setSttModelsList] = useState(undefined);
    const [llmProvidersList, setLlmProvidersList] = useState(undefined);

    const getActiveTtsModel = () => {
        if (ttsModelsList === undefined) return undefined;
        const active = ttsModelsList.find((m) => m.active);
        return active || null;
    };

    const getActiveSttModel = () => {
        if (sttModelsList === undefined) return undefined;
        const active = sttModelsList.find((m) => m.active);
        return active || null;
    };

    const getActiveLlmModel = () => {
        if (llmProvidersList === undefined) return undefined;

        const active = llmProvidersList.flatMap((p) =>
            p.models.filter((m) => m.active).map((m) => ({ ...p, ...m }))
        )[0];

        return active || null;
    };

    const buildFetchFunc = (kind, apiObj, setFunc) => {
        return async () => {
            setFunc(undefined);

            const response = await apiObj.getAll();
            if (isResponseOk(response)) {
                setFunc(response.data);
            } else {
                setFunc(null);
                showToast("Error al obtener modelos " + kind.toUpperCase(), response.data.detail, "red");
            }
        };
    };

    const fetchFunctions = {
        tts: buildFetchFunc("tts", ttsModels, setTtsModelsList),
        stt: buildFetchFunc("stt", sttModels, setSttModelsList),
        llm: buildFetchFunc("llm", llmModels, setLlmProvidersList),
    };

    useEffect(() => {
        Object.values(fetchFunctions).forEach((func) => func());
    }, []);

    return (
        <ModelsContext.Provider
            value={{
                activeTab,
                setActiveTab,

                getActiveTtsModel,
                getActiveSttModel,
                getActiveLlmModel,

                ttsModelsList,
                setTtsModelsList,
                sttModelsList,
                setSttModelsList,
                llmProvidersList,
                setLlmProvidersList,

                fetchFunctions,
            }}
        >
            {children}
        </ModelsContext.Provider>
    );
};
export const useModels = () => useContext(ModelsContext);
