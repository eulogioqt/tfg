import React, { useRef, useState, useEffect, forwardRef, useImperativeHandle } from "react";

const ChatTextArea = forwardRef(({ onSend }, ref) => {
    const [value, setValue] = useState("");
    const textAreaRef = useRef(null);

    useImperativeHandle(ref, () => ({
        send: () => handleSend(),
        clear: () => clearInput(),
        focus: () => focusTextArea(),
        getValue: () => value,
    }));

    const handleSend = () => {
        onSend(value);
        clearInput();
    };

    const clearInput = () => {
        setValue("");
        textAreaRef.current.style.height = "40px";
    };

    const focusTextArea = () => {
        textAreaRef.current?.focus();
        textAreaRef.current?.setSelectionRange(value.length, value.length);
    };

    const handleChange = (event) => setValue(event.target.value);
    const handleKeyDown = (event) => {
        if (event.key === "Enter") {
            event.preventDefault();
            handleSend();
        }
    };

    useEffect(() => {
        if (textAreaRef.current) {
            textAreaRef.current.style.height = "40px";
            textAreaRef.current.style.height = `${textAreaRef.current.scrollHeight}px`;
        }
    }, [value]);

    return (
        <>
            <textarea
                ref={textAreaRef}
                value={value}
                className="chat-textarea"
                placeholder="Escribe un mensaje..."
                onChange={handleChange}
                onKeyDown={handleKeyDown}
                rows="2"
            />

            <style>{`
                    .chat-textarea {
                        resize: none;
                        overflow-y: auto;
                        min-height: 40px;
                        max-height: 150px;
                        border: none;
                        outline: none;
                        width: 100%;
                        padding: 8px;
                        font-size: 14px;
                    }
                    .chat-textarea:focus {
                        box-shadow: none !important;
                    }
                `}</style>
        </>
    );
});

export default ChatTextArea;
