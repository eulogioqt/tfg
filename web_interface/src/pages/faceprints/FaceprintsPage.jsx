import React, { useEffect, useState } from "react";

import { useAPI } from "../../contexts/APIContext";
import { useEventBus } from "../../contexts/EventBusContext";
import { FACEPRINT_EVENT } from "../../contexts/WebSocketContext";

// hacer esta pagina mas chula
// poner los spinner de cargando cuando este cargando
// paginacion...

// poner los ROS INFO con las subscripciones y esas cosas en ros2web
// poner que con los websockets haya un mensaje del protoclo HRI de UPDATE o algo asi
// para cuando se meta una clase nueva o lo q sea para q la api vuelva a hacer fetch. Poenr tmb un boton de recargar
// manual. Ver que pasa cuando sales de la pagina y entras como guardar la data en un context o algo
// cuando llegue ese mensaje de update al websocketcontext, que se guarde en una lista y este vaya consumiendo los datos
// preguntar ideas a chatgpt pa hacer esto ultimo super mega pro

const FaceprintsPage = () => {
    const { faceprints, isResponseOk } = useAPI();
    const { subscribe } = useEventBus();

    const [data, setData] = useState([]);
    const [editingName, setEditingName] = useState(null);
    const [newName, setNewName] = useState("");

    const setFaceprints = (faceprints) => setData(faceprints);
    const addFaceprint = (faceprint) => setData(data => [...data, faceprint]);
    const updateFaceprint = (name, faceprint) => setData(data => data.map((item) => (item.name === name ? faceprint : item)));
    const deleteFaceprint = (name) => setData(data => data.filter((item) => item.name != name));

    useEffect(() => {
        const fetchData = async () => {
            const response = await faceprints.getAll();
            if (isResponseOk(response))
                setFaceprints(response.data)
        };

        fetchData();
    }, []);

    useEffect(() => {
        const processEvent = async (e) => {
            if (e.event == FACEPRINT_EVENT.CREATE) {
                const response = await faceprints.getById(e.name);
                if (isResponseOk(response))
                    addFaceprint(response.data);
            } else if (e.event == FACEPRINT_EVENT.UPDATE) {
                const response = await faceprints.getById(e.name);
                if (isResponseOk(response))
                    updateFaceprint(e.name, response.data);
            } else if (e.event == FACEPRINT_EVENT.DELETE) {
                const response = await faceprints.delete(e.name);
                if (isResponseOk(response))
                    deleteFaceprint(e.name);
            }
        };

        const unsubscribe = subscribe("ROS_MESSAGE_FACEPRINT_EVENT", processEvent);
        return () => unsubscribe();
    }, [])

    const handleDelete = async (name) => {
        if (!window.confirm(`¿Seguro que deseas eliminar a ${name}?`)) return;

        const response = await faceprints.delete(name);
        if (isResponseOk(response))
            deleteFaceprint(name);
    };

    const handleUpdate = async (oldName) => {
        if (newName.trim() === "") return;

        if (newName.trim() != oldName.trim()) {
            const response = await faceprints.update(oldName, { name: newName });
            if (isResponseOk(response))
                updateFaceprint(oldName, response.data);
        }

        setEditingName(null);
    };

    const startEditing = (name) => {
        setEditingName(name);
        setNewName(name);
    };

    return (
        <div className="container mt-5 dvh-100">
            <h2 className="mb-4 pt-4">Base de Datos de Rostros Reconocidos</h2>
            <div className="table-responsive">
                <table className="table table-striped table-hover align-middle">
                    <thead className="table-dark">
                        <tr>
                            <th>Imagen</th>
                            <th>Score</th>
                            <th>Nombre</th>
                            <th>Features</th>
                            <th>Veces Promediado</th>
                            <th>Fecha de Registro</th>
                            <th>Acciones</th>
                        </tr>
                    </thead>
                    <tbody>
                        {data &&
                            data.sort((a, b) => a.learning_date - b.learning_date).map((person) => (
                                <tr key={person.name}>
                                    <td>
                                        <img
                                            src={`data:image/jpg;base64,${person.face}`}
                                            onError={(e) => {
                                                e.target.onerror = null;
                                                e.target.src =
                                                    "https://t3.ftcdn.net/jpg/05/16/27/58/360_F_516275801_f3Fsp17x6HQK0xQgDQEELoTuERO4SsWV.jpg";
                                            }}
                                            alt="face"
                                            className="rounded"
                                            width={64}
                                            height={64}
                                        />
                                    </td>
                                    <td>{person.face_score.toFixed(2)}</td>
                                    <td>
                                        {editingName === person.name ? (
                                            <input
                                                type="text"
                                                className="form-control"
                                                value={newName}
                                                onChange={(e) => setNewName(e.target.value)}
                                            />
                                        ) : (
                                            person.name
                                        )}
                                    </td>
                                    <td>{person.features.length} vectores</td>
                                    <td>{person.size.join(", ")}</td>
                                    <td>{new Date(person.learning_date * 1000).toLocaleString()}</td>
                                    <td>
                                        {editingName === person.name ? (
                                            <button
                                                className="btn btn-success btn-sm me-2"
                                                onClick={() => handleUpdate(person.name)}
                                            >
                                                Guardar
                                            </button>
                                        ) : (
                                            <button
                                                className="btn btn-outline-primary btn-sm me-2"
                                                onClick={() => startEditing(person.name)}
                                            >
                                                <i className="bi bi-pencil"></i>
                                            </button>
                                        )}
                                        <button
                                            className="btn btn-outline-danger btn-sm"
                                            onClick={() => handleDelete(person.name)}
                                        >
                                            <i className="bi bi-trash"></i>
                                        </button>
                                    </td>
                                </tr>
                            ))}
                        {data.length === 0 && (
                            <tr>
                                <td colSpan="7" className="text-center py-3">
                                    No se han encontrado datos.
                                </td>
                            </tr>
                        )}
                    </tbody>
                </table>
            </div>
        </div>
    );
};

export default FaceprintsPage;
