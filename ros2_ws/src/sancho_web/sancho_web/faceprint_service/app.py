import uvicorn
from fastapi import FastAPI
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

from .v1 import router as v1_router

load_dotenv()

app = FastAPI(
    docs_url="/api/articles/docs",  # Documentación Swagger
    redoc_url="/api/articles/redoc",  # Documentación Redoc
    openapi_url="/api/articles/openapi.json"  # OpenAPI Schema
)
app.title = "Faceprints Service"
app.version = "1.0.0"
app.add_middleware(GZipMiddleware, minimum_size=1000)
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:5173"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(v1_router, prefix="/api/v1/faceprints")