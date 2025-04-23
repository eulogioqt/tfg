import React, { useState } from "react";

import SimpleModal from "../../../components/SimpleModal";

import { useToast } from "../../../contexts/ToastContext";
import { useAPI } from "../../../contexts/APIContext";

const NewFaceprintModal = ({ handleClose, isOpen }) => {
    const { showToast } = useToast();
    const { faceprints } = useAPI();

    const [selectedImage, setSelectedImage] = useState(undefined);
    const [name, setName] = useState("");

    const handleFileChange = (event) => {
        const file = event.target.files[0];
        if (file && file.type.startsWith("image/")) {
            setSelectedImage(file);
        } else {
            showToast("Archivo no válido", "Solo se permiten imágenes", "red");
        }
    };

    const convertToBase64 = (file) => {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onloadend = () => resolve(reader.result);
            reader.onerror = reject;
            reader.readAsDataURL(file);
        });
    };

    const uploadFace = async () => {
        const trimmedName = name.trim(); // igual que strip() en Python

        if (!selectedImage) {
            showToast("Imagen requerida", "Por favor, selecciona una imagen.", "red");
            return;
        }

        if (!trimmedName) {
            showToast("Nombre requerido", "Por favor, introduce un nombre válido.", "red");
            return;
        }

        try {
            const base64Image = await convertToBase64(selectedImage);

            const response = await faceprints.create({
                name: trimmedName,
                image: base64Image,
            });

            if (response.status >= 200 && response.status < 300) {
                showToast("Éxito", "La cara ha sido procesada correctamente.", "green");
                handleClose();
                setSelectedImage(undefined);
                setName("");
            } else {
                const errorText = response.data?.detail || "Error inesperado";
                showToast("Error", errorText, "red");
            }
        } catch (error) {
            const errorMsg = error?.response?.data?.detail || error?.message || "Error inesperado al subir la cara.";
            showToast("Error", errorMsg, "red");
        }
    };

    return (
        <SimpleModal name={"upload-face"} title="Subir nueva cara" handleClose={handleClose} isOpen={isOpen} zIndex={100}>
            <div className="mb-3">
                <label htmlFor="nameInput" className="form-label">Nombre</label>
                <input
                    type="text"
                    className="form-control"
                    id="nameInput"
                    placeholder="Introduce un nombre"
                    value={name}
                    onChange={(e) => setName(e.target.value)}
                />
            </div>

            <div className="mb-3">
                <label htmlFor="fileInput" className="btn btn-secondary w-100">
                    Seleccionar imagen
                </label>
                <input
                    type="file"
                    className="form-control visually-hidden"
                    id="fileInput"
                    accept="image/*"
                    onChange={handleFileChange}
                />
            </div>

            {selectedImage ? (
                <div className="d-flex flex-column align-items-center">
                    <img
                        src={URL.createObjectURL(selectedImage)}
                        alt="Vista previa"
                        style={{
                            minWidth: "256px",
                            maxWidth: "256px",
                            objectFit: "cover",
                            border: "1px solid #ddd",
                            borderRadius: "4px",
                            marginBottom: "8px",
                        }}
                    />
                    <span>{selectedImage.name}</span>
                </div>
            ) : (
                <span style={{ color: "#aaa" }}>Ninguna imagen seleccionada</span>
            )}

            <button className="btn btn-primary mt-3 w-100" onClick={uploadFace}>
                Subir cara
            </button>
        </SimpleModal>
    );
};

export default NewFaceprintModal;
