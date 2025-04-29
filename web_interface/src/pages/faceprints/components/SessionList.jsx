import React, { useState } from "react";

import SessionItem from "./SessionItem";

const SessionList = ({ faceprint, sessions }) => {
    const [currentPage, setCurrentPage] = useState(1);
    const perPage = 5;

    const sortedData = [...sessions].sort((a, b) => a.start_time - b.start_time);
    const totalPages = Math.ceil(sortedData.length / perPage);
    const pageData = sortedData.slice((currentPage - 1) * perPage, currentPage * perPage);

    return (
        <>
            <div className="row">
                {pageData && pageData.length > 0 ? (
                    pageData.map((session) => <SessionItem key={session.id} session={session} />)
                ) : (
                    <div className="alert alert-info" role="alert">
                        No hay ninguna sesion registrada para {faceprint.name} (ID {faceprint.id}).
                    </div>
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

export default SessionList;
