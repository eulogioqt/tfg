import { BrowserRouter as Router, Route, Routes, Navigate } from "react-router-dom";

import Header from "./components/Header";
import Home from "./pages/home/Home";
import Chat from "./pages/chat/Chat";

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
                        <Route path="/" element={wrap(<Home />)} />
                        <Route path="/chat" element={<Chat />} />
                        <Route path="*" element={<Navigate to="/" />} />
                    </Routes>
                </WebSocketProvider>
            </APIProvider>
        </Router>
    );
};

export default App;
