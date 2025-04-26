import React from "react";

import ActionModal from "../../../components/ActionModal";

const ConfirmDeleteFaceModal = ({ name, handleClose, action }) => {
    return (
        <ActionModal
            name={"confirm-delete-faceprint"}
            title={"Eliminar rostro"}
            isOpen={name !== undefined}
            handleClose={handleClose}
            action={action}
            buttonText={"Borrar"}
            buttonColor={"danger"}
        >
            <span style={{ display: name ? "block" : "none" }}>
                ¿Estás seguro de que quieres eliminar el rostro de <b>{name}</b>?
            </span>
        </ActionModal>
    );
};

export default ConfirmDeleteFaceModal;
