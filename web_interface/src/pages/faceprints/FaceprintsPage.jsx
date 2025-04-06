import React, { useEffect, useState } from "react";
import { useAPI } from "../../contexts/APIContext";

const FaceprintsPage = () => {
    const { faceprints } = useAPI();
    const [data, setData] = useState([]);
    const [editingName, setEditingName] = useState(null);
    const [newName, setNewName] = useState("");

    useEffect(() => {
        const fetchData = async () => {
            const response = await faceprints.getAll();
            if (response.status >= 200 && response.status < 300) setData(response.data);
        };
        fetchData();
    }, []);

    const handleDelete = async (name) => {
        if (!window.confirm(`¿Seguro que deseas eliminar a ${name}?`)) return;
        await faceprints.delete(name);
        setData(data.filter((item) => item.name !== name));
    };

    const startEditing = (name) => {
        setEditingName(name);
        setNewName(name);
    };

    const handleUpdate = async (oldName) => {
        if (newName.trim() === "") return;
        const updated = await faceprints.update(oldName, { name: newName });
        setData(data.map((item) => (item.name === oldName ? updated.data : item)));
        setEditingName(null);
    };
    console.log("data", data);
    return (
        <div className="container mt-5">
            <h2 className="mb-4">Base de Datos de Rostros Reconocidos</h2>
            <div className="table-responsive">
                <table className="table table-striped table-hover align-middle">
                    <thead className="table-dark">
                        <tr>
                            <th>Imagen</th>
                            <th>Nombre</th>
                            <th>Features</th>
                            <th>Veces Promediado</th>
                            <th>Fecha de Registro</th>
                            <th>Acciones</th>
                        </tr>
                    </thead>
                    <tbody>
                        {data &&
                            data.map((person) => (
                                <tr key={person.name}>
                                    <td>
                                        <img
                                            src="https://via.placeholder.com/64"
                                            alt="face"
                                            className="rounded"
                                            width={64}
                                            height={64}
                                        />
                                    </td>
                                    <td>
                                        {editingName === person.name ? (
                                            <input
                                                type="text"
                                                className="form-control"
                                                value={newName}
                                                onChange={(e) => setNewName(e.target.value)}
                                            />
                                        ) : (
                                            person.name
                                        )}
                                    </td>
                                    <td>{person.features.length} vectores</td>
                                    <td>{person.size.reduce((a, b) => a + b, 0)}</td>
                                    <td>{new Date(person.learning_date * 1000).toLocaleString()}</td>
                                    <td>
                                        {editingName === person.name ? (
                                            <button
                                                className="btn btn-success btn-sm me-2"
                                                onClick={() => handleUpdate(person.name)}
                                            >
                                                Guardar
                                            </button>
                                        ) : (
                                            <button
                                                className="btn btn-outline-primary btn-sm me-2"
                                                onClick={() => startEditing(person.name)}
                                            >
                                                <i class="bi bi-pencil"></i>
                                            </button>
                                        )}
                                        <button
                                            className="btn btn-outline-danger btn-sm"
                                            onClick={() => handleDelete(person.name)}
                                        >
                                            <i class="bi bi-trash"></i>
                                        </button>
                                    </td>
                                </tr>
                            ))}
                        {data.length === 0 && (
                            <tr>
                                <td colSpan="6" className="text-center py-3">
                                    No se han encontrado datos.
                                </td>
                            </tr>
                        )}
                    </tbody>
                </table>
            </div>
        </div>
    );
};

export default FaceprintsPage;
