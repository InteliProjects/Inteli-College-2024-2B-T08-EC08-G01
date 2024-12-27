# Backend
O backend é a parte de um sistema responsável por gerenciar a lógica de negócios, a manipulação de dados e a comunicação com o banco de dados ou serviços externos. Ele funciona como o "cérebro" de uma aplicação, processando solicitações enviadas pelo frontend e retornando respostas com os dados necessários. O backend é crucial para garantir que uma aplicação seja funcional, segura e capaz de lidar com diferentes tipos de interações e usuários.

No caso, este projeto está utilizando o FastAPI para o desenvolvimento do backend. O FastAPI é um framework de alto desempenho para construir APIs web com Python. Ele é conhecido por sua facilidade de uso, suporte a validações automáticas de dados e geração de documentação interativa (Swagger).

Nesse sentido, com esse framework houve a construção das rotas que estão sendo responsáveis por interagir com o banco de dados. Sendo assim, abaixo pode-se ter cada rota mais detalhada seu funcionamento:

`/listar/usuario`: é a rota responsável por retornar todos os usuários que estão cadastrados no banco de dados até o momento.

`/criar/usuario`: é a rota responsável por criar um novo usuário no sistema.

`/atualizar/usuario`: é a rota responsável por atualizar algum dado do usuário.

`/listar/log`: é a rota responsável por retornar todos os logs que estão cadastrados no banco de dados até o momento.

`/criar/log`: é a rota responsável por criar um novo log no sistema.

`/atualizar/log`: é a rota responsável por atualizar algum dado do log.

Com isso, todas essas rotas foram criadas para organizar a estrutura do projeto para que se possa ter registros do que foi feito durante o processo diário do software. Além disso, estas partes do backend até o momento feitas foram organizadas para receber dados do frontend e do Large Language Model (LLM) para serem processadas e gerar um resultado retornado para o usuário.

Por fim, para executar o sistema descrito será necessário seguir os passos abaixo:

1 - Abrir o terminal integrado no Visual Studio Code no caminho `src/backend` e digitar nele `sudo docker-compose up` para inicializar a aplicação do banco de dados.

2 - Abrir outro terminal integrado no Visual Studio Code no caminho `src/backend` e digitar `python3 main.py` para inicializar o backend da aplicação.

3 - Abrir o Postman e começar a executar as rotas para ver seu funcionamento. Isso pode ser feito, por exemplo se colocar no método `GET` digitar na URL do presente no Postman (nesse caso também é possível executar em um navegador) `http://0.0.0.0:8000/listar/log` que retornará todos os conteúdos presentes na tabela do log.

