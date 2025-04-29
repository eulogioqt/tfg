import React from "react";
import { useNavigate, useParams } from "react-router-dom";

const FaceprintDetailPage = () => {
    const { id } = useParams();
    
    const navigate = useNavigate();

    return (
        <div className="container mt-5">
            <button className="btn btn-secondary mt-4" onClick={() => navigate("/faceprints")}>Volver</button>

            <div className="d-flex justify-content-between align-items-center mb-4 pt-4">
                <h2>Faceprint con ID {id}</h2>
            </div>
        </div>
    )
}

export default FaceprintDetailPage;