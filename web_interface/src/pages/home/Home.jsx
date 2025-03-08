import { useState } from "react";

const Home = () => {
    const [count, setCount] = useState(0);

    return (
        <>
            <div className="container d-flex flex-column justify-content-center align-items-center vh-100">
                <button className="btn btn-primary" onClick={() => setCount((c) => c + 1)}>
                    Contar
                </button>
                <span className="badge bg-secondary mt-2 fs-6">{count}</span>
            </div>
        </>
    );
};

export default Home;
