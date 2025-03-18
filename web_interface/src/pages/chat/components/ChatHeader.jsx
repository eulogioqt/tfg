import React from "react";
import { useWindowSize, BREAKPOINTS } from "../../../hooks/useWindowSize";

import sanchoHead from "../../../assets/images/sancho_head.jpg";

const ChatHeader = ({ collapsed, setCollapsed, handleNewChat }) => {
    const { width } = useWindowSize();

    return (
        <div
            className="w-100 px-md-5 px-3 d-flex align-items-center justify-content-between"
            style={{ minHeight: "75px", height: "75px", borderBottom: "1px solid #EAEAEA" }}
        >
            <div className="d-flex align-items-center justify-content-between justify-content-md-start w-100">
                <button
                    className="btn btn-white me-3"
                    style={{ display: collapsed || width < BREAKPOINTS.MD ? "block" : "none" }}
                    onClick={() => setCollapsed((collapsed) => !collapsed)}
                >
                    <i className="bi bi-list"></i>
                </button>

                <div className="d-flex justify-content-center align-items-center">
                    <img
                        className="img-fluid rounded-circle"
                        src={sanchoHead}
                        alt="Sancho"
                        style={{
                            width: "50px",
                            height: "50px",
                            display: collapsed || width < BREAKPOINTS.MD ? "block" : "block",
                        }}
                    />

                    <span
                        className="fw-bold fs-2 ms-3"
                        style={{
                            whiteSpace: "nowrap",
                            display: collapsed || width < BREAKPOINTS.MD ? "block" : "none",
                        }}
                    >
                        Sancho
                    </span>
                </div>

                <button
                    className="btn btn-white text-start"
                    style={{ display: width < BREAKPOINTS.MD ? "block" : "none" }}
                    onClick={() => handleNewChat()}
                >
                    <i className="bi bi-chat-dots"></i>
                </button>
            </div>
        </div>
    );
};

export default ChatHeader;
