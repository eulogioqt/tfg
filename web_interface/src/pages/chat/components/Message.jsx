import React from "react";

const Message = ({ message }) => {
    const { text, id, isHuman } = message;

    const backgroundColor = isHuman ? "#E0E0E0" : "#FFFFFF";
    const shadowColor = isHuman ? "#C0C0C0" : "#DDDDDD";

    return (
        <div className={isHuman ? "d-flex justify-content-end" : ""}>
            <div
                className="d-inline-block mb-2"
                style={{
                    backgroundColor: backgroundColor,
                    border: "1px " + shadowColor + " solid",
                    borderRadius: "8px",
                    maxWidth: "70%",
                    padding: "8px",
                    wordWrap: "break-word",
                }}
            >
                {text}
            </div>
        </div>
    );
};

export default Message;
