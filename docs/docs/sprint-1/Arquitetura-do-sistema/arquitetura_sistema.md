# Arquitetura de Software

A definição da arquitetura de software é um passo fundamental no desenvolvimento de um sistema de entrega automatizada de medicamentos para hospitais. Esta arquitetura orienta a integração e a interação entre os componentes do sistema, considerando tanto os requisitos funcionais (relacionados ao comportamento do software) quanto os requisitos não funcionais (ligados à qualidade do sistema, como segurança, escalabilidade e manutenção). Com esse planejamento, buscamos assegurar que o sistema seja confiável, eficiente e capaz de se adaptar a mudanças futuras.

## Diagrama de Blocos - Solução Inicial Proposta
A seguir, estão descritos os elementos gerais que compõem a solução inicial da arquitetura do sistema para o robô entregador de medicamentos.

1. WhatsApp
- **Função**: Permitir que os usuários façam solicitações de entrega e recebam notificações sobre o status do robô.
- **Descrição**: A integração com o WhatsApp será feita através do cliente Twilio, que encaminhará as mensagens para o backend para processamento.

2. Cliente WhatsApp (Twilio)
- **Função**: Interface intermediária para processar mensagens enviadas e recebidas pelo WhatsApp.
- **Descrição**: Responsável por garantir a comunicação entre o WhatsApp e o backend do sistema, transmitindo as mensagens de maneira segura e em tempo real.
3. Backend (FastAPI)
- **Função**: Processar mensagens recebidas do usuário e gerenciar as requisições para o robô entregador.
- **Descrição**: Atua como o núcleo da aplicação, recebendo as solicitações do cliente WhatsApp, integrando-se com o LLM para interpretação dos comandos, e coordenando as instruções enviadas ao robô.
4. LLM (Large Language Model)
- **Função**: Processar o conteúdo da mensagem para identificar o comando solicitado.
- **Descrição**: Com base em inteligência artificial, o LLM interpreta a mensagem e responde com o comando mais apropriado em formato JSON, informando o tipo de ação e o destino da entrega.
5. Banco de Dados (SQLite)
- **Função**: Armazenar dados de localização, histórico de entregas e informações de status dos robôs.
- **Descrição**: O banco de dados registra a última localização do robô e mantém o histórico das entregas para otimizar o sistema, permitindo consultas rápidas de status e localização (suporta os requisitos RF02 e RF07).
6. Service TurtleBot
- **Função**: Serviço responsável pela comunicação direta com o robô.
- **Descrição**: Implementado em C++, permite o envio de instruções de movimentação e recebe feedback do robô em tempo real. Também interage com sensores e hardware para monitorar o ambiente.
7. Hardware (Sensores e Controladores do Robô)
- **Função**: Detectar e responder a objetos e mapear o ambiente.
- **Descrição**: Equipado com sensores para evitar obstáculos e mapear o espaço ao redor, garantindo que o robô consiga percorrer o ambiente de forma autônoma e segura (atende aos requisitos RF05 e RF06).

### Diagramas de bloco detalhados

Para trazer uma esquematização clara e detalhada do funcionamento do projeto, foi realizado dois diagramas detalhando a solução. Para uma melhor compreensão, foi divido em duas etapas, sendo a primeira delas a solicitação de um serviço e a segunda, a execução do comando que foi solicitado.git  

<div align="center">
![Diagrama de solicatação de um serviço](/img/diagrama-solicitacao.png)
*Fonte: Elaborado pelos autores*
</div>

<div align="center">
![Diagrama de Execução de um comando solicitado](/img/diagrama-execucao.png)
*Fonte: Elaborado pelos autores*
</div>
## Fluxo Geral do Sistema
**Solicitação de Serviço:** O usuário envia uma mensagem de solicitação de entrega pelo WhatsApp.
Processamento de Mensagem: O cliente Twilio encaminha a mensagem para o backend, que envia o conteúdo para o LLM.
**Comando Interpretação**: O LLM interpreta o comando e retorna com as informações necessárias para que o backend processe o pedido.
**Execução da Entrega**: O backend verifica a localização do robô mais próximo e aciona o Service TurtleBot para iniciar a entrega, enviando atualizações ao usuário conforme o robô se desloca.
**Confirmação de Entrega**: O robô chega ao destino, o usuário é notificado e o banco de dados é atualizado com as informações da entrega.