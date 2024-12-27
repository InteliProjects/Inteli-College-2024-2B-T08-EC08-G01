Para rodar o banco de dados primeiramente comente no docker-compose.yml a parte do backend.

Após isso construa o container dando `docker compose up build`

Agora entre com as credencias presentes no `.env`.

Crie um banco chamado `cecia`
Na parte do host coloque `db`


```sql
Criação da tabela de usuario:

CREATE TABLE IF NOT EXISTS usuario (
            id SERIAL PRIMARY KEY,
            nome VARCHAR(100) NOT NULL,
            senha VARCHAR(100) NOT NULL,
            setor VARCHAR(100)
        );
        
Criação da tablea de logs:

CREATE TABLE IF NOT EXISTS log (
            id SERIAL PRIMARY KEY,
            origem_dado VARCHAR(100) NOT NULL,
            destino_dado VARCHAR(100) NOT NULL,
            id_usuario INTEGER,
            concluido INTEGER NOT NULL,
            FOREIGN KEY (id_usuario) REFERENCES usuario (id)
        );


Criação da tabela de posicoes:

CREATE TABLE IF NOT EXISTS posicoes (
            id SERIAL PRIMARY KEY,
            posicao_x VARCHAR(100) NOT NULL,
            posicao_y VARCHAR(100) NOT NULL,
            posicao_z VARCHAR(100) NOT NULL,

        );

```