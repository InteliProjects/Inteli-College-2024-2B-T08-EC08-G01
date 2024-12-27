import psycopg2
from psycopg2 import sql

# Configurações do banco de dados
DB_HOST = "localhost"  # Endereço do banco de dados
DB_PORT = "5432"       # Porta configurada no Docker Compose
DB_USER = "usuario"  # Substitua por ${POSTGRES_USER}
DB_PASSWORD = "senha"  # Substitua por ${POSTGRES_PASSWORD}
DB_NAME = "eu_banco"    # Banco inicial do Docker Compose

# Conexão ao banco inicial para criar o novo banco de dados
try:
    conn = psycopg2.connect(
        host=DB_HOST,
        port=DB_PORT,
        user=DB_USER,
        password=DB_PASSWORD,
        dbname=DB_NAME
    )
    conn.autocommit = True
    cursor = conn.cursor()
    
    # Criar o banco de dados "cecia" se não existir
    cursor.execute("SELECT 1 FROM pg_database WHERE datname = 'cecia2'")
    exists = cursor.fetchone()
    if not exists:
        cursor.execute("CREATE DATABASE cecia2")
        print("Banco de dados 'cecia' criado com sucesso.")
    else:
        print("Banco de dados 'cecia' já existe.")
    
    cursor.close()
    conn.close()

    # Conectar ao novo banco de dados "cecia" para criar tabelas
    conn = psycopg2.connect(
        host=DB_HOST,
        port=DB_PORT,
        user=DB_USER,
        password=DB_PASSWORD,
        dbname="cecia"
    )
    cursor = conn.cursor()

    # Criar tabela "usuario"
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS usuario (
            id SERIAL PRIMARY KEY,
            nome VARCHAR(100) NOT NULL,
            senha VARCHAR(100) NOT NULL,
            id_sessao_watson VARCHAR(100)
        );
    """)
    print("Tabela 'usuario' criada com sucesso.")

    # Criar tabela "log"
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS log (
            id SERIAL PRIMARY KEY,
            origem_dado VARCHAR(100) NOT NULL,
            destino_dado VARCHAR(100) NOT NULL,
            id_usuario INTEGER,
            concluido INTEGER NOT NULL,
            FOREIGN KEY (id_usuario) REFERENCES usuario (id)
        );
    """)
    print("Tabela 'log' criada com sucesso.")

    # Confirmar alterações
    conn.commit()

except Exception as e:
    print(f"Erro ao configurar o banco de dados: {e}")
finally:
    if 'cursor' in locals():
        cursor.close()
    if 'conn' in locals():
        conn.close()
