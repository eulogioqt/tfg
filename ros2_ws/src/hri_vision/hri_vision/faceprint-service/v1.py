import uvicorn
import json
import time
import rclpy
from fastapi import APIRouter, HTTPException, Query, Request, Path, Depends
from fastapi.responses import JSONResponse
from fastapi import FastAPI
from ros_client_node import RosClientNode

app = FastAPI()

rclpy.init()
ros_node = RosClientNode()

# hacer bien los endpoints:
# poner esto con el app y demas
# y el nodo ros bien

# get all, te da todos los users en el formato del pydantic (service nuevo)
# get by name, te da solo uno (service nuevo)
# put para cambiar nombre (service recognition/training)
# delete para borrar clase (service recognition/training)
# con chatgpt se hace en un plis plas

# hacer todo con las buenas practicas de ing web

@app.get("/faceprints")
def get_faceprints():
    try:
        datos = ros_node.pedir_faceprints()
        return JSONResponse(
            status_code=200,
            content=datos,
            headers={"Accept-Encoding": "gzip", "X-Total-Count": str(1)}
        )
    except Exception as e:
        return {"error": f"Error: {str(e)}"}

if __name__ == "__main__":
    uvicorn.run(app, host="localhost", port=8000)