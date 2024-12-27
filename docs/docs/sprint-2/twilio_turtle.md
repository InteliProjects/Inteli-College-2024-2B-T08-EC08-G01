# Controle do TurtleBot3 via WhatsApp com Twilio (Beta)

## Introdução ao Twilio [1]

**Twilio** é uma plataforma de comunicação em nuvem que fornece APIs para enviar e receber mensagens, fazer chamadas telefônicas, vídeos e até mesmo integrar-se ao WhatsApp. A Twilio facilita a implementação de interações de mensagens automáticas, chamadas e comunicações em aplicativos, permitindo que empresas e desenvolvedores criem experiências de usuário personalizadas.

### Como Funciona o Twilio para WhatsApp [2]

1. **Configuração do Sandbox**: Twilio oferece um ambiente sandbox que permite testes com WhatsApp antes de aplicar para um número de telefone oficial.
2. **Webhook de Mensagens**: Para interagir com mensagens de WhatsApp, Twilio usa webhooks, que são URLs onde as mensagens enviadas pelo usuário são recebidas e tratadas.
3. **Respostas Automáticas**: Quando o webhook recebe uma mensagem, ele a processa (neste caso, pelo Flask) e envia uma resposta automática de volta ao WhatsApp do usuário.

## Passo a Passo para Integrar o TurtleBot3 com WhatsApp Usando Twilio e Ngrok

### Requisitos

- **Webots**: Ambiente de simulação para robôs. (Inicialmente, no futuro será substituído por um TurtleBot3 real).
- **Twilio API**: Conta Twilio configurada para WhatsApp.
- **Ngrok**: Ferramenta para expor seu servidor Flask local para a internet.
- **ROS 2 e TurtleBot3**: Configurado para receber comandos através do ROS 2.
- **Python com Flask**: Servidor que receberá mensagens e controlará o robô.

### 1. Executar o Webots

- Abra o **Webots** e configure o ambiente de simulação para o TurtleBot3.
- Certifique-se de que o TurtleBot3 esteja corretamente configurado para responder aos comandos ROS enviados via tópicos.

### 2. Executar o Código do Repositório

- Tenha certeza de que o código do Flask está configurado e inclui os comandos para controlar o TurtleBot3 via ROS 2 usando `rclpy`.
- Execute o servidor Flask com o comando:

  ```bash
  python nome_do_arquivo.py
  ```

- Verifique se o servidor está rodando corretamente, geralmente acessando `http://127.0.0.1:5000` (ou outra porta definida) e se as rotas estão respondendo conforme esperado.

### 3. Executar o Ngrok

- Inicie o **ngrok** para expor o servidor Flask à internet, permitindo que a Twilio acesse o seu webhook.
- No terminal, execute:

  ```bash
  ngrok http 5000
  ```

  Aqui, `5000` é a porta padrão do Flask. Verifique se está usando a porta correta.

- Ngrok fornecerá um link público, como `https://xxxx.ngrok.io`, que é o endereço a ser utilizado no Twilio.

### 4. Configurar o Webhook no Twilio Console

- Acesse a sua conta no [Twilio Console](https://www.twilio.com/console).
- Vá para **Messaging** > **Try it Out** > **Send a WhatsApp message** para configurar o Sandbox do WhatsApp.
- No campo de **Webhook**, insira o link gerado pelo Ngrok, seguido da rota de webhook configurada no Flask. Por exemplo:

  ```
  https://xxxx.ngrok.io/bot
  ```

- Salve as configurações.

### 5. Enviar Mensagens para o Bot

- No seu WhatsApp, envie mensagens para o número fornecido pelo Twilio.
- Teste comandos como "andar", "parar", e "girar" para verificar se o TurtleBot3 responde corretamente aos comandos.

## Dependências e Requisitos

- Python pacotes:
  - Flask
  - Twilio
  - requests
  - ROS 2 (rclpy)
  - geometry_msgs
- Conta Twilio com credenciais para integração.

## Melhorias Futuras
- Adicionar suporte para mais comandos do TurtleBot3.
- Melhorar o tratamento de mensagens de voz.
- Implementar autenticação para o uso do robô.

## Referências

[1] [Twilio](https://www.twilio.com/)
[2] [Twilio API for WhatsApp](https://www.twilio.com/docs/whatsapp/api)