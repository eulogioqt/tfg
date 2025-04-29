import React, { useEffect, useState } from "react";
import { useNavigate, useParams } from "react-router-dom";

import { useFaceprints } from "../../contexts/FaceprintsContext";
import { useAPI } from "../../contexts/APIContext";
import { useToast } from "../../contexts/ToastContext";

const FaceprintDetailPage = () => {
    const { id } = useParams();
    
    const { showToast} = useToast();
    const { getFaceprint } = useFaceprints();
    const { faceprints } = useAPI();

    const [faceprint, setFaceprint] = useState(undefined);

    const navigate = useNavigate();

    useEffect(() => {
        const getActualFaceprint = async () => {
            const faceprint = getFaceprint(id);

            if (faceprint) setFaceprint(faceprint);
            else {
                const response = await faceprints.getById(id);
                if (isResponseOk(response)) setFaceprint(response.data)
                else showToast("Error", response.data.detail, "red");
            }
        }

        getActualFaceprint();
    }, [])

    return (
        <div className="container mt-5">
            <button className="btn btn-secondary mt-4" onClick={() => navigate("/faceprints")}>Volver</button>

            {!faceprint ? (
                    <div className="text-center py-5">
                        <div className="spinner-border text-primary" role="status" />
                    </div>
                ) : (
                <>
                    <div className="d-flex justify-content-between align-items-center mb-4 pt-4">
                        <h2>{faceprint.name}</h2>
                    </div>
                </>
            )}
        </div>
    )
}

export default FaceprintDetailPage;