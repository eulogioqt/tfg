import { useNavigate } from "react-router-dom";
import sanchoSrc from "/sancho.jpg";
import Footer from "../../components/Footer";

const HomePage = () => {
    const navigate = useNavigate();

    return (
        <div>
            {/* Hero principal */}
            <section className="bg-light text-center py-5 px-3 border-bottom shadow-sm" style={{marginTop: "76px"}}>
                <div className="container">
                    <h1 className="display-4 fw-bold mb-3">Bienvenido a <span className="text-primary">SanchoHub</span></h1>
                    <p className="lead text-muted mb-4">
                        Plataforma inteligente de interacción humano-robot con reconocimiento facial, conversación natural y gestión remota.
                    </p>
                    <button className="btn btn-primary btn-lg me-2" onClick={() => navigate("/chat")}>
                        <i className="bi bi-robot me-2"></i>Habla con Sancho
                    </button>
                    <button className="btn btn-outline-secondary btn-lg" onClick={() => navigate("/models")}>
                        <i className="bi bi-gear me-2"></i>Gestionar Modelos
                    </button>
                </div>
            </section>

            {/* Características clave */}
            <section className="container py-5">
                <div className="row text-center">
                    <div className="col-md-4 mb-4">
                        <i className="bi bi-person-bounding-box fs-1 text-primary mb-3"></i>
                        <h4>Reconocimiento Facial</h4>
                        <p className="text-muted">Sancho detecta y reconoce rostros en tiempo real, con identificación persistente y base de datos editable desde la web.</p>
                    </div>
                    <div className="col-md-4 mb-4">
                        <i className="bi bi-chat-dots fs-1 text-primary mb-3"></i>
                        <h4>Conversación Natural</h4>
                        <p className="text-muted">Sistema de diálogo basado en LLMs e integración con Whisper para transcripción y respuesta hablada con TTS.</p>
                    </div>
                    <div className="col-md-4 mb-4">
                        <i className="bi bi-cpu fs-1 text-primary mb-3"></i>
                        <h4>Gestión Modular</h4>
                        <p className="text-muted">Control total desde la interfaz: carga y descarga de modelos de voz, STT y LLM. Todo sincronizado con ROS 2.</p>
                    </div>
                </div>
            </section>

            {/* Capturas y evidencia */}
            <section className="bg-white border-top border-bottom py-5">
                <div className="container">
                    <h3 className="text-center fw-bold mb-5">Sancho en acción</h3>
                    <div className="row g-4 justify-content-center">
                        <div className="col-md-4">
                            <img src={sanchoSrc} className="img-fluid rounded shadow-sm" alt="Interfaz de rostros" />
                        </div>
                        <div className="col-md-4">
                            <img src={sanchoSrc} className="img-fluid rounded shadow-sm" alt="Interfaz de chat" />
                        </div>
                        <div className="col-md-4">
                            <img src={sanchoSrc} className="img-fluid rounded shadow-sm" alt="Gestión de modelos" />
                        </div>
                    </div>
                </div>
            </section>

            {/* Sobre el TFG */}
            <section className="container py-5">
                <h3 className="fw-bold text-center mb-4">Sobre este proyecto</h3>
                <p className="text-muted text-center col-md-10 offset-md-1">
                    Este Trabajo de Fin de Grado presenta un sistema de interacción humano-robot (HRI) completamente funcional
                    construido sobre ROS 2 y controlado mediante una interfaz web moderna. Sancho reconoce rostros, transcribe audio,
                    genera respuestas en lenguaje natural y permite al usuario gestionar los modelos de IA que lo hacen posible.
                    Todo ello desde cualquier navegador.
                </p>
            </section>
        </div>
    );
};

export default HomePage;
