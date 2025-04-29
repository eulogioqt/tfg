import { BrowserRouter as Router, Route, Routes, Navigate } from "react-router-dom";

import Header from "./components/Header";
import HomePage from "./pages/home/HomePage";
import ChatPage from "./pages/chat/ChatPage";
import FaceprintListPage from "./pages/faceprints/FaceprintListPage";
import FaceprintDetailPage from "./pages/faceprints/FaceprintDetailPage";

import WebSocketVideoViewer from "./components/WebSocketVideoViewer";
import LoadingScreen from "./components/LoadingScreen";

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
            <LoadingScreen />

            <Routes>
                <Route path="/" element={wrap(<HomePage />)} />
                <Route path="/faceprints" element={wrap(<FaceprintListPage />)} />
                <Route path="/faceprints/:id" element={wrap(<FaceprintDetailPage/>)} />
                <Route path="/chat" element={wrap(<ChatPage />)} />
                <Route path="*" element={<Navigate to="/" />} />
            </Routes>
        </Router>
    );
};

export default App;
