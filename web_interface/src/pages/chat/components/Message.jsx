import React from "react";

const Message = ({ text, isHuman }) => {
    const backgroundColor = isHuman ? "#D9FDD3" : "#FFFFFF";
    const shadowColor = isHuman ? "#A3D8A4" : "#E2E2E2";

    return (
        <div className={isHuman ? "d-flex justify-content-end" : ""}>
            <div
                className="d-inline-block mb-2"
                style={{
                    backgroundColor: backgroundColor,
                    borderRadius: "8px",
                    maxWidth: "80%",
                    padding: "8px",
                    boxShadow: `1px 1px ${shadowColor}`,
                }}
            >
                {text}
            </div>
        </div>
    );
};

export default Message;
