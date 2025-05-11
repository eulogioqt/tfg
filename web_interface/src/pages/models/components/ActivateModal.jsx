import React from "react";
import ActionModal from "../../../components/ActionModal";

const ActivateModal = ({ show, model, selectedSpeaker, setSelectedSpeaker, onClose, onConfirm }) => {
    if (!show || !model) return null;

    return (
        <ActionModal
            name="activate-model"
            title={`Activar modelo "${model.model}"`}
            isOpen={show}
            handleClose={onClose}
            action={onConfirm}
            buttonText="Activar"
            buttonColor="primary"
            size="md"
        >
            <label className="form-label">Selecciona un hablante</label>
            <select
                className="form-select"
                value={selectedSpeaker}
                onChange={(e) => setSelectedSpeaker(e.target.value)}
            >
                <option value="">Selecciona...</option>
                {model.speakers.map((spk) => (
                    <option key={spk} value={spk}>
                        {spk}
                    </option>
                ))}
            </select>
        </ActionModal>
    );
};

export default ActivateModal;
