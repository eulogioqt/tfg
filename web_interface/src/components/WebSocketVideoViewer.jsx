import React, { useEffect, useRef, useState } from "react";

import { useWebSocket } from "../contexts/WebSocketContext";
import { useWindowSize } from "../hooks/useWindowSize";

const WebSocketVideoViewer = () => {
    const { displayData } = useWebSocket();
    const { width } = useWindowSize();

    const [imgDimensions, setImgDimensions] = useState(undefined);

    const videoRef = useRef(null);

    useEffect(() => {
        if (displayData.IMAGE && !imgDimensions) {
            // Hacer algo para por si cambia de video, o que siempre ese calculando esto
            const img = new Image();
            img.src = "data:image/jpeg;base64," + displayData.IMAGE;

            img.onload = () => {
                setImgDimensions({ width: img.width, height: img.height });
            };
        }
    }, [displayData.IMAGE]);

    return (
        <div>
            {imgDimensions && (
                <img
                    className="position-absolute border border-danger border-5"
                    style={{
                        bottom: "32px",
                        right: "32px",
                        zIndex: "40",
                        objectFit: "contain",
                    }}
                    ref={videoRef}
                    src={"data:image/jpeg;base64," + displayData.IMAGE}
                    width={width * 0.2 < 120 ? 120 : width * 0.2}
                    height={(width * 0.2 < 120 ? 120 : width * 0.2) / (imgDimensions.width / imgDimensions.height)}
                />
            )}
        </div>
    );
};

export default WebSocketVideoViewer;
