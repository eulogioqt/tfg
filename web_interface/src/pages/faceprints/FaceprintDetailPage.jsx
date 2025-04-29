import React, { useEffect, useState } from "react";
import { useNavigate, useParams } from "react-router-dom";

import ConfirmDeleteFaceModal from "./components/ConfirmDeleteFaceModal";

import { useFaceprints } from "../../contexts/FaceprintsContext";
import { useAPI } from "../../contexts/APIContext";
import { useToast } from "../../contexts/ToastContext";

const FaceprintDetailPage = () => {
    const { id } = useParams();

    const { showToast } = useToast();
    const { getFaceprint, doUpdateFaceprint, doDeleteFaceprint } = useFaceprints();
    const { faceprints, isResponseOk } = useAPI();

    const [faceprint, setFaceprint] = useState(undefined);
    const [isDeleteModalOpen, setisDeleteModalOpen] = useState(false);
    const [isEditing, setIsEditing] = useState(false);
    const [newName, setNewName] = useState("");

    const navigate = useNavigate();

    useEffect(() => {
        const getActualFaceprint = async () => {
            const faceprint = getFaceprint(id);

            if (faceprint) {
                setFaceprint(faceprint);
                setNewName(faceprint.name);
            } else {
                const response = await faceprints.getById(id);
                if (isResponseOk(response)) {
                    setFaceprint(response.data);
                    setNewName(response.data.name);
                } else {
                    showToast("Error", response.data.detail, "red");
                }
            }
        };

        getActualFaceprint();
    }, []);

    const handleDelete = async (id) => {
        const response = await doDeleteFaceprint(id);
        if (isResponseOk(response)) navigate("/faceprints");
    }

    const handleUpdate = async (id, oldName, newName) => {
        if (newName.trim() === "") return;
        if (newName.trim() !== oldName.trim()) {
            const response = await doUpdateFaceprint(id, newName);
            if (isResponseOk(response)) faceprint.name = newName;
        }
        setIsEditing(false);
    };

    if (!faceprint) {
        return (
            <div className="text-center mt-5 py-5">
                <div className="spinner-border text-primary" role="status" />
            </div>
        );
    }

    return (
        <>
            <ConfirmDeleteFaceModal
                faceprint={faceprint}
                isOpen={isDeleteModalOpen}
                handleClose={() => setisDeleteModalOpen(false)}
                action={() => handleDelete(faceprint.id)}
            />

            <div className="container mt-5">
                <button className="btn btn-secondary my-4" onClick={() => navigate("/faceprints")}>
                    Volver
                </button>

                <div className="card shadow p-4 mb-5">
                    <div className="d-flex flex-column align-items-center">
                        <img
                            src={`data:image/jpg;base64,${faceprint.face}`}
                            alt={faceprint.name}
                            className="rounded-circle mb-3"
                            style={{ width: 256, height: 256, objectFit: "cover" }}
                        />

                        <div className="d-flex align-items-center mb-3">
                            {isEditing ? (
                                <>
                                    <input
                                        type="text"
                                        className="form-control me-2"
                                        style={{ width: "250px" }}
                                        value={newName}
                                        onChange={(e) => setNewName(e.target.value)}
                                    />
                                    <button className="btn btn-success btn-sm me-2" onClick={() => handleUpdate(faceprint.id, faceprint.name, newName)}>
                                        Guardar
                                    </button>
                                    <button className="btn btn-outline-secondary btn-sm" onClick={() => setIsEditing(false)}>
                                        Cancelar
                                    </button>
                                </>
                            ) : (
                                <>
                                    <h2 className="mb-0 me-3">{faceprint.name}</h2>
                                    <button
                                        className="btn btn-outline-primary btn-sm"
                                        onClick={() => setIsEditing(true)}
                                    >
                                        <i className="bi bi-pencil" />
                                    </button>
                                </>
                            )}
                        </div>
                    </div>

                    <div className="mt-4">
                        <div className="row g-3">
                            <div className="col-md-6">
                                <div className="bg-secondary-subtle p-3 rounded">
                                    <h6 className="text-muted mb-1">ID</h6>
                                    <p className="fw-bold mb-0">{faceprint.id}</p>
                                </div>
                            </div>
                            <div className="col-md-6">
                                <div className="bg-secondary-subtle p-3 rounded">
                                    <h6 className="text-muted mb-1">Fecha de registro</h6>
                                    <p className="fw-bold mb-0">{new Date(faceprint.learning_date * 1000).toLocaleString()}</p>
                                </div>
                            </div>
                            <div className="col-md-6">
                                <div className="bg-secondary-subtle p-3 rounded">
                                    <h6 className="text-muted mb-1">Número de vectores</h6>
                                    <p className="fw-bold mb-0">{faceprint.features.length}</p>
                                </div>
                            </div>
                            <div className="col-md-6">
                                <div className="bg-secondary-subtle p-3 rounded">
                                    <h6 className="text-muted mb-1">Confianza de detección</h6>
                                    <p className="fw-bold mb-0">{faceprint.face_score.toFixed(2)}</p>
                                </div>
                            </div>
                        </div>

                        <div className="text-center mt-5">
                            <button className="btn btn-outline-danger" onClick={() => setisDeleteModalOpen(true)}>
                                <i className="bi bi-trash me-2" /> Borrar
                            </button>
                        </div>
                    </div>
                </div>

                <div className="mt-5">
                    <h4 className="mb-3">Resumen de actividad</h4>
                    <div className="alert alert-info" role="alert">
                        Próximamente: sesiones, detecciones, gráficas de actividad...
                    </div>
                </div>
            </div>
        </>
    );
};

export default FaceprintDetailPage;
