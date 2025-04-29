import React, { useState } from "react";

import { useFaceprints } from "../../contexts/FaceprintsContext";

import NewFaceprintModal from "./components/NewFaceprintModal";
import FaceprintItem from "./components/FaceprintItem";

const FaceprintListPage = () => {
    const {
        doAddFaceprint,
        fetchFaceprintsData,
        loadingFaceprints,
        faceprintsData,
    } = useFaceprints();

    const [isOpenFaceModal, setIsOpenFaceModal] = useState(false);

    const [currentPage, setCurrentPage] = useState(1);
    const perPage = 12;

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
                        <div className="row">
                            {pageData && pageData.length > 0 ? pageData.map((faceprint) => (
                                <FaceprintItem key={faceprint.id} faceprint={faceprint}/>
                            )) : (
                                <p className="text-center text-secondary">No se encontraron rostros.</p>
                            )}
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

export default FaceprintListPage;
