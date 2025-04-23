import React from "react";

const SimpleModal = ({ name, title, handleClose, isOpen, size = "lg", zIndex = 100, children }) => {
    return (
        <div
            id={name + "-"}
            onClick={handleClose}
            className={`modal ${isOpen ? 'show' : ''}`}
            style={{ display: isOpen ? 'block' : 'none', backgroundColor: "rgb(0.5,0.5,0.5,0.5)", zIndex: zIndex }}
            tabIndex="-1"
        >
            <div className={"modal-dialog modal-" + size} role="document" onClick={(e) => e.stopPropagation()}>
                <div className="modal-content">
                    <div className="modal-header">
                        <h5 className="modal-title" id={name + "-modal-label"}>
                            {title}
                        </h5>
                        <button type="button" className="btn-close" onClick={handleClose}></button>
                    </div>
                    <div className="modal-body">
                        {children}
                    </div>
                    <div className="modal-footer">
                        <button type="button" className="btn btn-secondary" onClick={handleClose}>Cerrar</button>
                    </div>
                </div>
            </div>
        </div >
    );
};

export default SimpleModal;