import React, { useState } from "react";
import { useNavigate } from "react-router-dom";

const Sidebar = ({ children }) => {
    const [collapsed, setCollapsed] = useState(false);
    const navigate = useNavigate();

    return (
        <div className="d-flex">
            <div
                className="sidebar bg-light border-end p-3 d-flex flex-column"
                style={{
                    transform: collapsed ? "translateX(-11.25rem)" : "translateX(0)",
                    transition: "transform 0.3s ease",
                    width: "16rem",
                    position: "fixed",
                    height: "100vh",
                }}
            >
                <div>
                    <div className="d-flex justify-content-between align-items-center">
                        <span className="fw-bold fs-2" style={{ marginLeft: "12px", whiteSpace: "nowrap" }}>
                            Sancho
                        </span>

                        <button className="btn btn-light" onClick={() => setCollapsed(!collapsed)}>
                            <i className="bi bi-list"></i>
                        </button>
                    </div>

                    <div className="d-flex flex-column mt-3">
                        <button className="btn btn-light text-start">
                            <i className="bi bi-chat-dots me-2"></i>
                            <span>Nuevo Chat</span>
                        </button>

                        <button className="btn btn-light text-start" onClick={() => navigate("/")}>
                            <i className="bi bi-house-door me-2"></i>
                            <span>Inicio</span>
                        </button>
                    </div>
                </div>
            </div>

            <div
                style={{
                    width: collapsed ? "4.75rem" : "16rem",
                    transition: "width 0.3s ease",
                    height: "100vh",
                    overflow: "hidden",
                    flexShrink: 0,
                }}
            ></div>
        </div>
    );
};

export default Sidebar;
