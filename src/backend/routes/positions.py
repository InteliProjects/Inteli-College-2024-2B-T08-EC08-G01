from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel
from psycopg2 import sql
from conexao import connect_db

router = APIRouter()


@router.get("/listar")
async def listar_posicoes():
    try:
        conn = connect_db()
        query = "SELECT * FROM posicoes"
        posicoes = conn.execute(query).fetchall()

        # Transformando os resultados em uma lista de dicionários
        posicoes_formatados = [
            {
                "id": posicao[0],
                "lugar": posicao[1],
                "posicao_x": posicao[2],
                "posicao_y": posicao[3],
                "posicao_z": posicao[4],
            }
            for posicao in posicoes
        ]

        return {"Posições": posicoes}

    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Erro ao listar posições: {str(e)}"
        )


@router.post("/criar")
def cria_usuario(
    id: int = Query(..., description="ID da posição"),
    lugar: str = Query(..., description="Nome da localização da posição"),
    posicao_x: str = Query(..., description="Valor da posição X"),
    posicao_y: str = Query(..., description="Valor da posição Y"),
    posicao_z: str = Query(..., description="Valor da posição Z"),
):
    try:
        conn = connect_db()
        cursor = conn.cursor()

        # Comando SQL para inserir dados na tabela posicoes
        query = "INSERT INTO posicoes (id, lugar, posicao_x, posicao_y, posicao_z) VALUES (?, ?, ?, ?, ?)"

        # Executando o comando SQL
        cursor.execute(query, (id, lugar, posicao_x, posicao_y, posicao_z))
        conn.commit()  # Confirma a transação

        return {"message": f"Posição {lugar} registrada com sucesso!"}

    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Erro ao registrar posição: {str(e)}"
        )


@router.put("/atualizar/{id}")
async def atualiza_posicao(
    id: int,
    lugar: str = Query(None, description="Nome da localização da posição"),
    posicao_x: str = Query(None, description="Valor da posição X"),
    posicao_y: str = Query(None, description="Valor da posição Y"),
    posicao_z: str = Query(None, description="Valor da posição Z"),
):
    try:
        conn = connect_db()
        cursor = conn.cursor()

        campos = []
        valores = []

        if lugar:
            campos.append("lugar = ?")
            valores.append(lugar)

        if posicao_x:
            campos.append("posicao_x = ?")
            valores.append(posicao_x)

        if posicao_y:
            campos.append("posicao_y = ?")
            valores.append(posicao_y)

        if posicao_z:
            campos.append("posicao_z = ?")
            valores.append(posicao_z)

        if not campos:
            raise HTTPException(status_code=400, detail="Nenhum dado para atualizar.")

        valores.append(id)  # Adiciona o ID no final da lista de valores

        query = f"UPDATE posicoes SET {', '.join(campos)} WHERE id = ?"
        cursor.execute(query, tuple(valores))  # Executa a query
        conn.commit()

        if cursor.rowcount == 0:  # Verifica se alguma linha foi alterada
            raise HTTPException(status_code=404, detail="Posição não encontrada.")

        return {"message": f"Posição com ID {id} atualizada com sucesso!"}

    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Erro ao atualizar posição: {str(e)}"
        )


@router.delete("/deletar/{id}")
async def deletar_posicao(id: int):
    try:
        conn = connect_db()
        cursor = conn.cursor()

        # Comando SQL para deletar a posição pelo ID
        query = "DELETE FROM posicoes WHERE id = ?"
        cursor.execute(query, (id,))
        conn.commit()

        if cursor.rowcount == 0:  # Verifica se alguma linha foi deletada
            raise HTTPException(status_code=404, detail="Posição não encontrada.")

        return {"message": f"Posição com ID {id} deletada com sucesso!"}

    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Erro ao deletar posição: {str(e)}"
        )
