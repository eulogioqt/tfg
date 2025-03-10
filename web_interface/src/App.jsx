import { BrowserRouter as Router, Route, Routes, Navigate } from "react-router-dom";

import Header from "./components/Header";
import Home from "./pages/home/Home";
import Chat from "./pages/chat/Chat";

import { WebSocketProvider } from "./contexts/WebSocketContext";

const App = () => {
    const wrap = (page) => (
        <>
            <Header />
            {page}
        </>
    );

    return (
        <Router>
            <WebSocketProvider>
                <Routes>
                    <Route path="/" element={wrap(<Home />)} />
                    <Route path="/chat" element={<Chat />} />
                    <Route path="*" element={<Navigate to="/" />} />
                </Routes>
            </WebSocketProvider>
        </Router>
    );
};

export default App;
