from fastapi import APIRouter, HTTPException, Query
from psycopg2 import sql
from conexao import connect_db
from datetime import datetime
import time
import requests

router = APIRouter()
ipAdd = '0.0.0.0'

@router.get("/listar")
def listar_log():
    try:
        conn = connect_db()
        # cursor = conn.cursor()

        # Comando SQL para buscar todos os logs
        query = "SELECT id, origem_dado, destino_dado, id_usuario, concluido FROM log"
        # conn.execute(query)

        logs = conn.execute(query).fetchall()

        logs_formatados = [
            {
                "id": logs[0],
                "origem_dado": logs[1],
                "destino_dado": logs[2],
                "id_usuario": logs[3],
                "concluido": logs[4]
            }
            for logs in logs
        ]

        return {"logs": logs_formatados}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Erro ao listar logs: {str(e)}")
    


@router.post("/criar")
def cria_log(
    origem_dado: str = Query(..., description="Origem do log"),
    destino_dado: str = Query(..., description="Destino do log"),
    id_usuario: int = Query(..., description="ID da sessão do Watson"),
    concluido: int = Query(..., description="Se o pedido foi concluído")
):
    try:
        conn = connect_db()
        # cursor = conn.cursor()
        # Comando SQL para inserir dados na tabela log
        query ="INSERT INTO log (origem_dado, destino_dado, id_usuario, concluido) VALUES (?, ?, ?, ?)"
        # Executando o comando SQL
        conn.execute(query, (origem_dado, destino_dado, id_usuario,concluido))
        conn.commit()  # Confirma a transação
        # cursor.close()
        # conn.close()
        try:
            r = requests.get("http://0.0.0.0:8000/log/adicionar_fila")
            return {"message": f"Log {id} registrado com sucesso e adicionado na fila!"}
        except:
            return {"message": f"Log {id} registrado com sucesso!"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Erro ao registrar usuário: {str(e)}")
    


@router.put("/atualizar/{id}")
def atualiza_usuario(
    id: int,
    origem_dado: str = Query(None, description="Origem do log"),
    destino_dado: str = Query(None, description="Destino do log"),
    horario: str = Query(None, description="Horaio do log"),
    id_usuario: int = Query(None, description="ID do usuario"),
    concluido: int = Query(..., description="Se o pedido foi concluído")
):
    try:
        conn = connect_db()
            # cursor = conn.cursor()


        campos = []
        # valores = []

        if origem_dado:
            campos.append(f"origem_dado = {origem_dado}")
            # valores.append(origem_dado)

        if destino_dado:
            campos.append(f"destino_dado = {destino_dado}")
            # valores.append(destino_dado)
    
        if horario:
            try:
                horario = datetime.strptime(horario, "%H:%M:%S").time()
                campos.append(f"horario = {horario}")
                # valores.append(horario)
            except ValueError:
                raise HTTPException(status_code=400, detail="Formato inválido para horário. Use HH:MM:SS.")
    
        if id_usuario:
            try:
                id_usuario = int(id_usuario)
                campos.append(f"id_usuario = {id_usuario}")
                # valores.append(id_usuario)
            except ValueError:
                raise HTTPException(status_code=400, detail="ID do usuário deve ser um número inteiro.")

        if concluido:
            campos.append(f"concluido = {concluido}")
            # valores.append(concluido)

        # if concluido:
        #     campos.append("concluido = %s")
        #     valores.append(concluido)

        if not campos:
            raise HTTPException(status_code=400, detail="Nenhum dado para atualizar.")
        
        # valores.append(id)
        query = f"""
            UPDATE log SET {", ".join(campos)} WHERE id = {id}
        """
        # conn.execute(query, tuple(valores))
        conn.execute(query)
        conn.commit()
        return {"message": f"Log com ID {id} atualizado com sucesso!"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Erro ao atualizar usuário: {str(e)}")
       

@router.get("/adicionar_fila")
async def add():
    try:
        logs = listar_log()["logs"]
        fila = {}
        for i in range(len(logs)):
            if logs[i]["concluido"] == 0:
                fila[logs[i]["id"]] = [logs[i]["origem_dado"],logs[i]["destino_dado"]]
        r = requests.post(f"http://{ipAdd}:5000/add_seq", json={"info":fila})
        return {"message": f"Logs adicionados na fila com sucesso!"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Erro ao adicionar os logs na fila: {str(e)}")
