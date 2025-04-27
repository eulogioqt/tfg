import { useContext, createContext, useState, useEffect } from "react";

import { useAPI } from "./APIContext";
import { useEventBus } from "./EventBusContext";
import { useToast } from "./ToastContext";
import { useLoadingScreen } from "../components/LoadingScreen";

import { FACEPRINT_EVENT } from "./WebSocketContext";

const FaceprintsContext = createContext();

export const FaceprintsProvider = ({ children }) => {
    const { showToast } = useToast();
    const { subscribe } = useEventBus();
    const { faceprints, isResponseOk } = useAPI();
    const { withLoading } = useLoadingScreen();

    const [faceprintsData, setFaceprints] = useState([]);
    const [loadingFaceprints, setLoadingFaceprints] = useState(true);

    const addFaceprint = (fc) => setFaceprints((prev) => [...prev, fc]);
    const updateFaceprint = (name, fc) => setFaceprints((prev) => prev.map((item) => (item.name === name ? fc : item)));
    const deleteFaceprint = (name) => setFaceprints((prev) => prev.filter((item) => item.name !== name));

    const fetchFaceprintsData = async () => {
        setLoadingFaceprints(true);

        const response = await faceprints.getAll();
        if (isResponseOk(response)) {
            setFaceprints(response.data);
        } else {
            showToast("Error", "No se han podido cargar los datos", "red");
        }

        setLoadingFaceprints(false);
    };

    useEffect(() => {
        fetchFaceprintsData();
    }, []);

    useEffect(() => {
        const processEvent = async (e) => {
            if (e.event === FACEPRINT_EVENT.CREATE || e.event === FACEPRINT_EVENT.UPDATE) {
                const response = await faceprints.getById(e.name);
                if (isResponseOk(response)) {
                    if (e.event === FACEPRINT_EVENT.CREATE) {
                        addFaceprint(response.data);
                        showToast("Nueva persona aprendida", "Se ha aprendido a la persona " + e.name, "green");
                    } else {
                        updateFaceprint(e.name, response.data);
                    }
                } else {
                    showToast("Error", response.data.detail, "red");
                }
            } else if (e.event === FACEPRINT_EVENT.DELETE) {
                deleteFaceprint(e.name);
                showToast("Persona eliminada", "Se ha eliminado a la persona " + e.name, "green");
            }
        };

        const unsubscribe = subscribe("ROS_MESSAGE_FACEPRINT_EVENT", processEvent);
        return () => unsubscribe();
    }, []);

    const doAddFaceprint = async (name, base64Image) => {
        const response = await withLoading(() =>
            faceprints.create({
                name: name,
                image: base64Image,
            })
        );

        if (isResponseOk(response)) {
            addFaceprint(response.data);
            if (response.status === 208)
                showToast(
                    "Persona conocida",
                    `Ya te conocía ${response.data.name}, pero he reforzado mi aprendizaje.`,
                    "blue"
                );
            else
                showToast(
                    "Éxito",
                    `La cara ha sido procesada y se ha aprendido a ${response.data.name} correctamente.`,
                    "green"
                );
        } else {
            showToast("Error", response.data.detail, "red");
        }
    };

    const doUpdateFaceprint = async (oldName, newName) => {
        const response = await withLoading(() => faceprints.update(oldName, { name: newName }));
        if (isResponseOk(response)) {
            updateFaceprint(oldName, response.data);
            showToast("Persona editada", "Has editado a la persona " + newName + " satisfactoriamente", "green");
        } else {
            showToast("Error", response.data.detail, "red");
        }
    };

    const doDeleteFaceprint = async (name) => {
        const response = await withLoading(() => faceprints.delete(name));
        if (isResponseOk(response)) {
            deleteFaceprint(name);
            showToast("Persona eliminada", "Has eliminado a la persona " + name + " satisfactoriamente", "green");
        } else {
            showToast("Error", response.data.detail, "red");
        }
    };

    return (
        <FaceprintsContext.Provider
            value={{
                doAddFaceprint,
                doUpdateFaceprint,
                doDeleteFaceprint,
                fetchFaceprintsData,

                loadingFaceprints,
                faceprintsData,
            }}
        >
            {children}
        </FaceprintsContext.Provider>
    );
};

export const useFaceprints = () => useContext(FaceprintsContext);
