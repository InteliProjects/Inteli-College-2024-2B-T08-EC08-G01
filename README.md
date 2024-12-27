# CECIA
Repositório do grupo **CECIA**

## Descrição do Projeto
Este projeto foi desenvolvido em parceria com a  International Business Machines Corporation (IBM) para a construção de um robô autônomo voltado à automação de processos na área hospitalar. O principal objetivo é auxiliar os usuários em tarefas rotineiras dentro do hospital, otimizando fluxos de trabalho e melhorando a eficiência do ambiente hospitalar.

A solução inclui um robô autônomo projetado para realizar a entrega de medicamentos mediante solicitação. Quando o usuário faz um pedido, o robô se desloca até a farmácia do hospital, recolhe o medicamento e o entrega no local especificado pelo solicitante.

Para a execução desse projeto, o Turtlebot3 com a versão Humble foi escolhido como a plataforma robótica. Ele será responsável por percorrer as rotas dentro do hospital, garantindo a entrega eficiente dos medicamentos. Além disso, a interface de interação com o usuário será feita por meio do WhatsApp, permitindo que os usuários façam requisições de medicamentos de forma simples e prática, com opções de seleção bem estruturadas de texto e fala.

Por fim, esse projeto representa uma inovação no uso de robôs em ambientes hospitalares, ao integrar tecnologias de automação com ferramentas de comunicação amplamente utilizadas, como o WhatsApp.



## Objetivos do Projeto
Desenvolver um robô autônomo que, utilizando inteligência artificial, seja
capaz de realizar tarefas rotineiras de cuidado e monitoramento de pacientes. O robô
ajudará na adesão ao tratamento, lembretes de medicação, monitoramento de sinais
vitais, emissão de alertas em caso de emergências e fornecerá suporte emocional por
meio de interações verbais, reduzindo a carga de trabalho dos profissionais de saúde.


## 👨‍🎓 Integrantes: 
- <a href="https://www.linkedin.com/in/cec%C3%ADlia-alonso-gon%C3%A7alves-3aa4bb271/">Cecilia Gonçalves</a>
- <a href="https://www.linkedin.com/in/eduardo-henrique-dos-santos/">Eduardo Santos</a>
- <a href="https://www.linkedin.com/in/fernando-vasconcellos-/">Fernando Vasconcellos</a>
- <a href="https://www.linkedin.com/in/gabriel-gallo-m-coutinho-443809232/">Gabriel Gallo</a>
- <a href="https://www.linkedin.com/in/guilherme-ferreira-linhares-8638411a1/">Guilherme Linhares</a>
- <a href="https://www.linkedin.com/in/josevalencar/">José Alencar</a>
- <a href="https://www.linkedin.com/in/lidiamariano/">Lídia Mariano</a>
- <a href="https://www.linkedin.com/in/vitoria-novaes/">Vitória Novaes</a>

## 👩‍🏫 Professores:
### Orientador
- <a href="https://www.linkedin.com/in/rafaelmatsuyama/">Rafael Matsuyama</a>
### Instrutores
- <a href="https://www.linkedin.com/in/diogo-martins-gon%C3%A7alves-de-morais-96404732/">Diogo Morais</a>
- <a href="https://www.linkedin.com/in/gui-cestari/">Guilherme Cestari</a>
- <a href="https://www.linkedin.com/in/filipe-gon%C3%A7alves-08a55015b/">Filipe Gonçalves</a>
- <a href="https://www.linkedin.com/in/lisane-valdo/">Lisane Valdo</a> 
- <a href="https://www.linkedin.com/in/rodrigo-mangoni-nicola-537027158/">Rodrigo Nicolla</a>

## Estrutura de Pastas
```
2024-2B-T08-EC08-G01/
├── .github
├── .vscode
├── docs
|   ├── docs
|   ├── src
|   ├── static
├── src
│   ├── backend
│   │   ├── __pycache__
│   │   ├── routes
│   │   ├── script_banco
│   │   └── venv
│   ├── cli
│   ├── deprecated
│   ├── frontend
│   └── twilio
├── .gitignore
├── README.md
├── requisitos.txt

```

## Guia de instrução 

Para executar este projeto siga os passos abaixo:

1. Clone este repositório.

2. Vá para a pasta `src/backend`.

3. Execute o comando no terminal integrado do Visual Studio Code `docker compose build`.

4. Agora no terminal integrado execute `docker compose up`.

## Documentação

Para acessar a nossa documentação, clique [aqui](https://github.com/Inteli-College/2024-2B-T08-EC08-G01/tree/main/docs/docs)!

## 📋 Licença/License
<!-- <p xmlns:cc="http://creativecommons.org/ns#" xmlns:dct="http://purl.org/dc/terms/"><a property="dct:title" rel="cc:attributionURL" href="https://github.com/Inteli-College/2024-2B-T08-EC08-G01">Robô autônomo hospitalar</a> by <span property="cc:attributionName">Inteli, Cecília Gonçalves, Eduardo Santos, Fernando Vasconcellos Gabriel Gallo, Guilherme Linhares, José Alencar, Lídia Mariano, Vitória Novaes</span> is licensed under <a href="https://creativecommons.org/licenses/by/4.0/?ref=chooser-v1" target="_blank" rel="license noopener noreferrer" style="display:inline-block;">CC BY 4.0<img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/cc.svg?ref=chooser-v1" alt=""><img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/by.svg?ref=chooser-v1" alt=""></a></p> -->

<div xmlns:cc="http://creativecommons.org/ns#" xmlns:dct="http://purl.org/dc/terms/">
    <a property="dct:title" rel="cc:attributionURL" href="https://github.com/Inteli-College/2024-2B-T08-EC08-G01">
        Robô autônomo hospitalar
    </a>
    <span>
        by
    </span>
    <span property="cc:attributionName">
        <a href="https://www.inteli.edu.br/">Inteli</a>,
        <a href="https://www.linkedin.com/in/cec%C3%ADlia-alonso-gon%C3%A7alves-3aa4bb271/">Cecília Gonçalves</a>,
        <a href="https://www.linkedin.com/in/eduardo-henrique-dos-santos/">Eduardo Santos</a>,
        <a href="https://www.linkedin.com/in/fernando-vasconcellos-/">Fernando Vasconcellos</a>
        <a href="https://www.linkedin.com/in/gabriel-gallo-m-coutinho-443809232/">Gabriel Gallo</a>,
        <a href="https://www.linkedin.com/in/guilherme-ferreira-linhares-8638411a1/">Guilherme Linhares</a>,
        <a href="https://www.linkedin.com/in/josevalencar/">José Alencar</a>,
        <a href="https://www.linkedin.com/in/lidiamariano/">Lídia Mariano</a>,
        <a href="https://www.linkedin.com/in/vitoria-novaes/">Vitória Novaes</a>,
    </span> 
    <span>
        is licensed under
    </span>
    <a href="https://creativecommons.org/licenses/by/4.0/?ref=chooser-v1" target="_blank" rel="license noopener noreferrer" style="display:inline-block;">
        CC BY 4.0
        <img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/cc.svg?ref=chooser-v1" alt="Creative Commons">
        <img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/by.svg?ref=chooser-v1" alt="Attribution">
    </a>
</div>






## 🗃 Histórico de lançamento

- 0.1 -  15/10/2024
  - Início do Projeto

  
