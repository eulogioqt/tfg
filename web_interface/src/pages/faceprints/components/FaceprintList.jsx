import React, { useState } from "react";

import FaceprintItem from "./FaceprintItem";

const FaceprintList = ({ faceprints }) => {
    const [currentPage, setCurrentPage] = useState(1);
    const perPage = 12;

    const sortedData = [...faceprints].sort((a, b) => a.learning_date - b.learning_date);
    const totalPages = Math.ceil(sortedData.length / perPage);
    const pageData = sortedData.slice((currentPage - 1) * perPage, currentPage * perPage);

    return (
        <>
            <div className="row">
                {pageData && pageData.length > 0 ? (
                    pageData.map((faceprint) => <FaceprintItem key={faceprint.id} faceprint={faceprint} />)
                ) : (
                    <p className="text-center text-secondary">No se encontraron rostros.</p>
                )}
            </div>

            {totalPages > 1 && (
                <nav className="mt-4 d-flex justify-content-center">
                    <ul className="pagination">
                        {[...Array(totalPages)].map((_, index) => (
                            <li key={index} className={`page-item ${currentPage === index + 1 ? "active" : ""}`}>
                                <button className="page-link" onClick={() => setCurrentPage(index + 1)}>
                                    {index + 1}
                                </button>
                            </li>
                        ))}
                    </ul>
                </nav>
            )}
        </>
    );
};

export default FaceprintList;
