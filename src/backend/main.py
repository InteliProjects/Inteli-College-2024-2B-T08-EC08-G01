from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
from routes import usuario
from routes import log
from routes import positions

app = FastAPI()

# Configuração do CORS
app.add_middleware(
    CORSMiddleware,
    allow_methods=["*"],
    allow_headers=["*"],
    allow_credentials=True,
    allow_origins=["*"],
)


app.include_router(usuario.router, prefix="/usuario", tags=["Usuario"])
app.include_router(positions.router, prefix="/positions", tags=["Positions"])
app.include_router(log.router, prefix="/log", tags=["log"])
# app.include_router(twilio.router, prefix="/twilio", tags=["twilio"])

if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
