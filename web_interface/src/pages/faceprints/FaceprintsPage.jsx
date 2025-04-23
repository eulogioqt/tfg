import React, { useEffect, useState } from "react";

import { useAPI } from "../../contexts/APIContext";
import { useEventBus } from "../../contexts/EventBusContext";
import { useToast } from "../../contexts/ToastContext";

import { FACEPRINT_EVENT } from "../../contexts/WebSocketContext";
import NewFaceprintModal from "./components/NewFaceprintModal";

const FaceprintsPage = () => {
    const { showToast } = useToast();
    const { faceprints, isResponseOk } = useAPI();
    const { subscribe } = useEventBus();

    const [data, setData] = useState([]);
    const [loading, setLoading] = useState(true);
    const [editingName, setEditingName] = useState(null);
    const [newName, setNewName] = useState("");
    const [isOpenFaceModal, setIsOpenFaceModal] = useState(false);

    const [currentPage, setCurrentPage] = useState(1);
    const perPage = 5;

    const fetchData = async () => {
        setLoading(true);

        const response = await faceprints.getAll();
        if (isResponseOk(response)) setData(response.data);

        setLoading(false);
    };

    useEffect(() => {
        fetchData();
    }, []);

    useEffect(() => {
        const processEvent = async (e) => {
            if (e.event === FACEPRINT_EVENT.CREATE || e.event === FACEPRINT_EVENT.UPDATE) {
                const response = await faceprints.getById(e.name);
                if (isResponseOk(response)) {
                    setData((prev) =>
                        e.event === FACEPRINT_EVENT.CREATE
                            ? [...prev, response.data]
                            : prev.map((item) => (item.name === e.name ? response.data : item))
                    );
                }
            } else if (e.event === FACEPRINT_EVENT.DELETE) {
                setData((prev) => prev.filter((item) => item.name !== e.name));
            }
        };

        const unsubscribe = subscribe("ROS_MESSAGE_FACEPRINT_EVENT", processEvent);
        return () => unsubscribe();
    }, []);

    const handleDelete = async (name) => {
        if (!window.confirm(`¿Seguro que deseas eliminar a ${name}?`)) return;

        const response = await faceprints.delete(name);
        if (isResponseOk(response)) {
            setData((prev) => prev.filter((item) => item.name !== name));
        }
    };

    const handleUpdate = async (oldName, newName) => {
        if (newName.trim() === "") return;

        if (newName.trim() !== oldName.trim()) {
            const response = await faceprints.update(oldName, { name: newName });
            if (isResponseOk(response)) {
                setData((prev) => prev.map((item) => (item.name === oldName ? response.data : item)));
            }
        }

        setEditingName(null);
    };

    const sortedData = [...data].sort((a, b) => a.learning_date - b.learning_date);
    const totalPages = Math.ceil(sortedData.length / perPage);
    const pageData = sortedData.slice((currentPage - 1) * perPage, currentPage * perPage);

    return (
        <>
            <NewFaceprintModal isOpen={isOpenFaceModal} handleClose={() => setIsOpenFaceModal(false)}/>

            <div className="container mt-5">
                <div className="d-flex justify-content-between align-items-center mb-4 pt-4">
                    <h2>Base de Datos de Rostros Reconocidos</h2>
                    <div>
                        <button className="btn btn-primary me-2" onClick={() => setIsOpenFaceModal(true)}>
                            <i className="bi bi-database-add me-2" /> Añadir cara
                        </button>
                        <button className="btn btn-outline-secondary" onClick={fetchData}>
                            <i className="bi bi-arrow-clockwise me-2" /> Recargar
                        </button>
                    </div>
                </div>

                {loading ? (
                    <div className="text-center py-5">
                        <div className="spinner-border text-primary" role="status" />
                    </div>
                ) : (
                    <>
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
                                    {pageData.length === 0 ? (
                                        <tr>
                                            <td colSpan="7" className="text-center py-3">
                                                No se han encontrado datos.
                                            </td>
                                        </tr>
                                    ) : (
                                        pageData.map((person) => (
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
                                                            onClick={() => handleUpdate(person.name, newName)}
                                                        >
                                                            Guardar
                                                        </button>
                                                    ) : (
                                                        <button
                                                            className="btn btn-outline-primary btn-sm me-2"
                                                            onClick={() => {
                                                                setEditingName(person.name);
                                                                setNewName(person.name);
                                                            }}
                                                        >
                                                            <i className="bi bi-pencil" />
                                                        </button>
                                                    )}
                                                    <button
                                                        className="btn btn-outline-danger btn-sm"
                                                        onClick={() => handleDelete(person.name)}
                                                    >
                                                        <i className="bi bi-trash" />
                                                    </button>
                                                </td>
                                            </tr>
                                        ))
                                    )}
                                </tbody>
                            </table>
                        </div>

                        {totalPages > 1 && (
                            <nav className="mt-4 d-flex justify-content-center">
                                <ul className="pagination">
                                    {[...Array(totalPages)].map((_, index) => (
                                        <li
                                            key={index}
                                            className={`page-item ${currentPage === index + 1 ? "active" : ""}`}
                                        >
                                            <button className="page-link" onClick={() => setCurrentPage(index + 1)}>
                                                {index + 1}
                                            </button>
                                        </li>
                                    ))}
                                </ul>
                            </nav>
                        )}
                    </>
                )}
            </div>
        </>
    );
};

export default FaceprintsPage;
