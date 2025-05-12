import React, {createContext, useContext, useEffect, useState} from "react";
import { useToast } from "./ToastContext";
import { useAPI } from "./APIContext";

const ModelsContext = createContext();

export const ModelsProvider = ({children}) => {
    const { ttsModels, sttModels, llmModels, isResponseOk } = useAPI();
    const { showToast } = useToast();

    const [activeTab, setActiveTab] = useState("tts");
    const [ttsModelsList, setTtsModelsList] = useState([]);
    const [sttModelsList, setSttModelsList] = useState([]);
    const [llmProvidersList, setLlmProvidersList] = useState([]);

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
                setLlmProvidersList(response.data);
            } else {
                showToast("Error al obtener modelos LLM", response.data.detail, "red");
            }
        };

        fetchLLM();
    }, []);

    return (
        <ModelsContext.Provider
            value={{
                activeTab,
                setActiveTab,
                
                ttsModelsList,
                setTtsModelsList,
                sttModelsList,
                setSttModelsList,
                llmProvidersList,
                setLlmProvidersList
            }}
        >
            {children}
        </ModelsContext.Provider>
    );

}
export const useModels = () => useContext(ModelsContext);