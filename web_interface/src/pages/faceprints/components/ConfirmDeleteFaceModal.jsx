import React from "react";

import ActionModal from "../../../components/ActionModal";

const ConfirmDeleteFaceModal = ({ id, handleClose, action }) => {
    return (
        <ActionModal
            name={"confirm-delete-faceprint"}
            title={"Eliminar rostro"}
            isOpen={id !== undefined}
            handleClose={handleClose}
            action={action}
            buttonText={"Borrar"}
            buttonColor={"danger"}
        >
            <span style={{ display: id ? "block" : "none" }}>
                ¿Estás seguro de que quieres eliminar el rostro de la persona con id <b>{id}</b>?
            </span>
        </ActionModal>
    );
};

export default ConfirmDeleteFaceModal;
