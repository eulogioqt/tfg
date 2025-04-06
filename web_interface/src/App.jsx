import { BrowserRouter as Router, Route, Routes, Navigate } from "react-router-dom";

import Header from "./components/Header";
import HomePage from "./pages/home/HomePage";
import ChatPage from "./pages/chat/ChatPage";
import FaceprintsPage from "./pages/faceprints/FaceprintsPage";

import WebSocketVideoViewer from "./components/WebSocketVideoViewer";

import { WebSocketProvider } from "./contexts/WebSocketContext";
import { APIProvider } from "./contexts/APIContext";

const App = () => {
    const wrap = (page) => (
        <>
            <Header />
            {page}
        </>
    );

    return (
        <Router>
            <APIProvider>
                <WebSocketProvider>
                    <WebSocketVideoViewer />

                    <Routes>
                        <Route path="/" element={wrap(<HomePage />)} />
                        <Route path="/chat" element={<ChatPage />} />
                        <Route path="/faceprints" element={<FaceprintsPage />} />
                        <Route path="*" element={<Navigate to="/" />} />
                    </Routes>
                </WebSocketProvider>
            </APIProvider>
        </Router>
    );
};

export default App;
