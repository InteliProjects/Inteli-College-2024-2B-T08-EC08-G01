from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel
from psycopg2 import sql
from conexao import connect_db

router = APIRouter()

class Login(BaseModel):
    nome: str
    senha: str


@router.get("/listar")
async def listar_usuarios():
    try:
        conn = connect_db()

        # Comando SQL para buscar todos os usuários
        # print(conn.execute("select * from usuario").fetchall())

        query = "SELECT id, nome, senha, setor FROM usuario"
        usuarios = conn.execute(query).fetchall()

        # Transformando os resultados em uma lista de dicionários
        usuarios_formatados = [
            {
                "id": usuario[0],
                "nome": usuario[1],
                "senha": usuario[2],
                "setor": usuario[3]
            }
            for usuario in usuarios
        ]

        return {"usuarios": usuarios_formatados}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Erro ao listar usuários: {str(e)}")


@router.post("/criar")
def cria_usuario(
    id: int = Query(..., description="ID do usuário"),
    nome: str = Query(..., description="Nome do usuário"),
    senha: str = Query(..., description="Senha do usuário"),
    setor: str = Query(..., description="ID da sessão do Watson")
):
    # conn = connect_db()

    try:
        conn = connect_db()
        # cursor = conn.cursor() 


        # Comando SQL para inserir dados na tabela usuario
        query = "INSERT INTO usuario (id, nome, senha, setor) VALUES (?, ?, ?, ?)"

        # Executando o comando SQL
        conn.execute(query, (id, nome, senha, setor))
        conn.commit()  # Confirma a transação

        return {"message": f"Usuário {nome} registrado com sucesso!"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Erro ao registrar usuário: {str(e)}")


@router.put("/atualizar/{id}")
async def atualiza_usuario(
    id: int,
    nome: str = Query(None, description="Nome do usuário"),
    senha: str = Query(None, description="Senha do usuário"),
    setor: str = Query(None, description="Setor do usuário")
):
    try:
        conn = connect_db()
        cursor = conn.cursor()

        campos = []
        valores = []

        if nome:
            campos.append("nome = ?")
            valores.append(nome)

        if senha:
            campos.append("senha = ?")
            valores.append(senha)
        
        if setor:
            campos.append("setor = ?")
            valores.append(setor)

        if not campos:
            raise HTTPException(status_code=400, detail="Nenhum dado para atualizar.")
        
        valores.append(id)  # Adiciona o ID no final da lista de valores

        query = f"UPDATE usuario SET {', '.join(campos)} WHERE id = ?"
        cursor.execute(query, tuple(valores))  # Executa a query
        conn.commit()

        if cursor.rowcount == 0:  # Verifica se alguma linha foi alterada
            raise HTTPException(status_code=404, detail="Usuário não encontrado.")

        return {"message": f"Usuário com ID {id} atualizado com sucesso!"}

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Erro ao atualizar usuário: {str(e)}")

        
@router.post("/login")
async def login_usuario(nome: str, senha: str):
    conn = connect_db()
    # cursor = conn.cursor()

    try:
        # Comando SQL para buscar todos os usuários
        query = "SELECT id, nome FROM usuario WHERE nome = ? AND senha = ?"
        conn.execute(query, (nome, senha))
        usuario = conn.fetchone()
        if usuario:
            return {"message": "Login feito com sucesso!", "id": usuario[0], "nome":usuario[1]}
        else:
            raise HTTPException(status_code=401, detail="Credenciais inválidas")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Erro ao processar login: {str(e)}")