import { BrowserRouter as Router, Route, Routes, Navigate } from "react-router-dom";

import Header from "./components/Header";
import HomePage from "./pages/home/HomePage";
import ChatPage from "./pages/chat/ChatPage";
import FaceprintsPage from "./pages/faceprints/FaceprintsPage";

import WebSocketVideoViewer from "./components/WebSocketVideoViewer";

const App = () => {
    const wrap = (page) => (
        <>
            <Header />
            {page}
        </>
    );

    return (
        <Router>
            <WebSocketVideoViewer />

            <Routes>
                <Route path="/" element={wrap(<HomePage />)} />
                <Route path="/faceprints" element={wrap(<FaceprintsPage />)} />
                <Route path="/chat" element={wrap(<ChatPage />)} />
                <Route path="*" element={<Navigate to="/" />} />
            </Routes>
        </Router>
    );
};

export default App;
