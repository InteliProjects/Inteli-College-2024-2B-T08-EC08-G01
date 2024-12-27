import sqlite3
import libsql_experimental as libsql

url = "libsql://ceciadb-cecia.turso.io"

auth_token = "eyJhbGciOiJFZERTQSIsInR5cCI6IkpXVCJ9.eyJpYXQiOjE3MzQ0NTI3NzUsImlkIjoiZTQ1ZjgyOGMtOTFlZi00ZmI5LTk4MjUtMjUwZDY2NzYxMGRmIn0.gaR7D7FGlCx9ueHVYqiFskHBIR76HphSa7iZo5LWbZQv4Z233N1WYAquQZS2oB2TPl3Gh3UAodf5F2Hi9oSiDg"



def criar_tabela(conexao):
    cursor = conexao.cursor()
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS usuario (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            nome TEXT NOT NULL,
            senha TEXT NOT NULL,
            setor TEXT NOT NULL
        )
    """)
    conexao.commit()

def inserir_usuario(conexao, nome, senha, setor):
    cursor = conexao.cursor()
    cursor.execute("""
        INSERT INTO usuario (nome, senha, setor) 
        VALUES (?, ?, ?)
    """, (nome, senha, setor))
    conexao.commit()
    print(f"Usuário '{nome}' inserido com sucesso.")

def main():
    # Conectar ao banco de dados
    try:
        conexao = libsql.connect("ceciadb.db", sync_url=url, auth_token=auth_token)
        print("Conexão bem-sucedida com o banco de dados Turso.")

        # Criar tabela (caso ainda não exista)
        criar_tabela(conexao)

        # Inserir dados na tabela
        usuarios = [
            ("João Silva", "senha123", "TI"),
            ("Maria Souza", "senha456", "RH"),
            ("Carlos Pereira", "senha789", "Financeiro")
        ]
        
        for nome, senha, setor in usuarios:
            inserir_usuario(conexao, nome, senha, setor)

    except sqlite3.Error as e:
        print(f"Erro ao conectar ao banco de dados: {e}")
    finally:
        if conexao:
            # conexao.close()
            print(conexao.execute("select * from usuario").fetchall())
            print("Conexão encerrada.")

            

if __name__ == "__main__":
    main()
