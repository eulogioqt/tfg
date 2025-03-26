import React, { useEffect, useRef, useState } from "react";
import { useWebSocket } from "../contexts/WebSocketContext";
import { useWindowSize, BREAKPOINTS } from "../hooks/useWindowSize";
import DraggableItem from "./DraggableItem";

const WebSocketVideoViewer = () => {
    const { displayData } = useWebSocket();
    const { width } = useWindowSize();

    const [imgDimensions, setImgDimensions] = useState(null);
    const [showVideo, setShowVideo] = useState(false);

    const timeoutRef = useRef(null);

    useEffect(() => {
        if (displayData.IMAGE) {
            const img = new Image();
            img.src = `data:image/jpeg;base64,${displayData.IMAGE}`;

            img.onload = () => {
                setImgDimensions({ width: img.width, height: img.height });
                setShowVideo(true);

                if (timeoutRef.current) clearTimeout(timeoutRef.current);
                timeoutRef.current = setTimeout(() => {
                    setShowVideo(false);
                }, 5000);
            };
        }

        return () => clearTimeout(timeoutRef.current);
    }, [displayData.IMAGE]);

    const calculatedWidth = Math.max(width * 0.2, 160);
    const calculatedHeight =
        imgDimensions && (calculatedWidth * imgDimensions.height) / imgDimensions.width;

    return (
        <DraggableItem>
            {showVideo && imgDimensions && (
                <img
                    className="position-absolute border border-dark bg-dark border-4 rounded shadow-lg"
                    style={{
                        zIndex: 40 * (width < BREAKPOINTS.MD ? 1 : 10),
                        objectFit: "contain",
                    }}
                    src={`data:image/jpeg;base64,${displayData.IMAGE}`}
                    width={calculatedWidth}
                    height={calculatedHeight}
                    alt="WebSocket Video"
                />
            )}
        </DraggableItem>
    );
};

export default WebSocketVideoViewer;
