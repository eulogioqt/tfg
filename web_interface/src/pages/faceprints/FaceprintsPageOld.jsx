import React, { useState } from "react";

import { useFaceprints } from "../../contexts/FaceprintsContext";

import NewFaceprintModal from "./components/NewFaceprintModal";
import ConfirmDeleteFaceModal from "./components/ConfirmDeleteFaceModal";

const FaceprintsPage = () => {
    const {
        doAddFaceprint,
        doUpdateFaceprint,
        doDeleteFaceprint,
        fetchFaceprintsData,
        loadingFaceprints,
        faceprintsData,
    } = useFaceprints();

    const [editingName, setEditingName] = useState(null);
    const [newName, setNewName] = useState("");

    const [isOpenFaceModal, setIsOpenFaceModal] = useState(false);
    const [deleteModalId, setDeleteModalId] = useState(undefined);

    const [currentPage, setCurrentPage] = useState(1);
    const perPage = 5;

    const handleDelete = async (id) => await doDeleteFaceprint(id);
    const handleUpdate = async (id, oldName, newName) => {
        if (newName.trim() === "") return;
        if (newName.trim() !== oldName.trim()) {
            await doUpdateFaceprint(id, newName);
        }
        setEditingName(null);
    };

    const sortedData = [...faceprintsData].sort((a, b) => a.learning_date - b.learning_date);
    const totalPages = Math.ceil(sortedData.length / perPage);
    const pageData = sortedData.slice((currentPage - 1) * perPage, currentPage * perPage);

    return (
        <>
            <NewFaceprintModal
                isOpen={isOpenFaceModal}
                handleClose={() => setIsOpenFaceModal(false)}
                doAddFaceprint={doAddFaceprint}
            />

            <ConfirmDeleteFaceModal
                id={deleteModalId}
                handleClose={() => setDeleteModalId(undefined)}
                action={() => handleDelete(deleteModalId)}
            />

            <div className="container mt-5">
                <div className="d-flex justify-content-between align-items-center mb-4 pt-4">
                    <h2>Base de Datos de Rostros Reconocidos</h2>
                    <div>
                        <button className="btn btn-primary me-2" onClick={() => setIsOpenFaceModal(true)}>
                            <i className="bi bi-database-add me-2" /> Añadir cara
                        </button>
                        <button className="btn btn-outline-secondary" onClick={fetchFaceprintsData}>
                            <i className="bi bi-arrow-clockwise me-2" /> Recargar
                        </button>
                    </div>
                </div>

                {loadingFaceprints ? (
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
                                        <th>id</th>
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
                                            <td colSpan="8" className="text-center py-3">
                                                No se han encontrado datos.
                                            </td>
                                        </tr>
                                    ) : (
                                        pageData.map((person) => (
                                            <tr key={person.id}>
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
                                                <td>{person.id}</td>
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
                                                            onClick={() =>
                                                                handleUpdate(person.id, person.name, newName)
                                                            }
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
                                                        onClick={() => setDeleteModalId(person.id)}
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
