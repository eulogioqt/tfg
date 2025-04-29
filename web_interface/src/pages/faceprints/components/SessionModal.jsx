import React from "react";
import SimpleModal from "../../../components/SimpleModal";
import { Line, Bar } from "react-chartjs-2";
import { Chart, LineElement, PointElement, LinearScale, CategoryScale, BarElement } from "chart.js";

Chart.register(LineElement, PointElement, LinearScale, CategoryScale, BarElement);

// Pequeño componente para gráficos
const SimpleChart = ({ title, labels, data, type = "line" }) => {
    const chartData = {
        labels,
        datasets: [
            {
                label: title,
                data,
            },
        ],
    };

    const options = {
        responsive: true,
        plugins: {
            legend: {
                display: false,
            },
        },
        scales: {
            y: {
                beginAtZero: true,
            },
        },
    };

    if (type === "bar") {
        return <Bar data={chartData} options={options} />;
    }
    return <Line data={chartData} options={options} />;
};

const calcStats = (arr) => {
    if (!arr.length) return {};
    const mean = arr.reduce((a, b) => a + b, 0) / arr.length;
    const sorted = [...arr].sort((a, b) => a - b);
    const median =
        sorted.length % 2 === 0
            ? (sorted[sorted.length / 2 - 1] + sorted[sorted.length / 2]) / 2
            : sorted[Math.floor(sorted.length / 2)];
    const min = Math.min(...arr);
    const max = Math.max(...arr);
    const std = Math.sqrt(arr.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / arr.length);
    return { mean, median, min, max, std };
};

const SessionModal = ({ session, handleClose, isOpen }) => {
    if (!session) return null;

    const duration = (parseFloat(session.end_time) - parseFloat(session.start_time)).toFixed(2);
    const numDetections = session.detections.length;

    const times = session.detections.map((d) => parseFloat(d[0]) - parseFloat(session.start_time));
    const detectionScores = session.detections.map((d) => d[1]);
    const classificationScores = session.detections.map((d) => d[2]);

    const detectionStats = calcStats(detectionScores);
    const classificationStats = calcStats(classificationScores);

    return (
        <SimpleModal
            name="session-info"
            title="Información sobre la sesión"
            handleClose={handleClose}
            isOpen={isOpen}
            zIndex={100}
        >
            <div className="mb-3">
                <h5 className="mb-2">Detalles generales</h5>
                <ul className="list-group list-group-flush mb-3">
                    <li className="list-group-item">ID de sesión: {session.id}</li>
                    <li className="list-group-item">Duración: {duration} segundos</li>
                    <li className="list-group-item">Detecciones: {numDetections}</li>
                    <li className="list-group-item">
                        Frecuencia media: {(numDetections / duration).toFixed(2)} detecciones/seg
                    </li>
                </ul>

                <h5 className="mb-2">Estadísticas de detección</h5>
                <ul className="list-group list-group-flush mb-3">
                    <li className="list-group-item">Score medio: {detectionStats.mean.toFixed(3)}</li>
                    <li className="list-group-item">Score mediano: {detectionStats.median.toFixed(3)}</li>
                    <li className="list-group-item">Score mínimo: {detectionStats.min.toFixed(3)}</li>
                    <li className="list-group-item">Score máximo: {detectionStats.max.toFixed(3)}</li>
                    <li className="list-group-item">Desviación estándar: {detectionStats.std.toFixed(3)}</li>
                </ul>

                <h5 className="mb-2">Estadísticas de clasificación</h5>
                <ul className="list-group list-group-flush mb-3">
                    <li className="list-group-item">Score medio: {classificationStats.mean.toFixed(3)}</li>
                    <li className="list-group-item">Score mediano: {classificationStats.median.toFixed(3)}</li>
                    <li className="list-group-item">Score mínimo: {classificationStats.min.toFixed(3)}</li>
                    <li className="list-group-item">Score máximo: {classificationStats.max.toFixed(3)}</li>
                    <li className="list-group-item">Desviación estándar: {classificationStats.std.toFixed(3)}</li>
                </ul>

                <h5 className="mb-2">Gráficos de evolución</h5>
                <div className="mb-4">
                    <SimpleChart title="Detection Score" labels={times} data={detectionScores} />
                </div>
                <div className="mb-4">
                    <SimpleChart title="Classification Score" labels={times} data={classificationScores} />
                </div>
            </div>
        </SimpleModal>
    );
};

export default SessionModal;
