# Função de chat com LLM 

No projeto desnvolvido pela equipe, desenvolvemos duas formas de adicionar um pedido para o robô: pela plataforma web e pelo WhatsApp. Na plataforma web, os pedidos são estruturados na requisição e enviados para a tabela que armazena os logs (pedidos) no nosso banco de dados, já no WhatsApp, primeiramente identificamos o tipo da mensagem (áudio ou texto), estruturamos uma mensagem para a API da OpenAI e ele nos retorna a localidade de destino a qual utilizamos para criar um novo pedido.

## Conversão de áudio para texto

Para fazermos a conversão de áudio para texto, utilizamos duas bibliotecas de Python: a [SpeechRecognition](https://pypi.org/project/SpeechRecognition/) e a [PySoundfile](https://pypi.org/project/PySoundFile/). 

A biblioteca PySoundfile é uma biblioteca que permite a edição de arquivos de áudio, permitindo a leitura e escrita dos bytes que compõem o arquivo de áudio. Na solução, a utilizamos para abrir um arquivo buffer de áudio com final ".ogg" que consultamos da API do Twilio (biblioteca para comunicação com o WhatsApp) e escrever sua informação em um outro arquivo com extensão ".wav", passo necessário para o funcionamento correto da biblioteca SpeechRecognition.

Já a biblioteca SpeechRecognition lê o arquivo de áudio e envia as informações para uma API aberta que identifica o texto no áudio (no projeto, escolhemos a API do Google). Com as informações recebidas pelo Twilio escritas no arquivo com extensão ".wav", inicializamos a SpeechRecognition, utilizamos um método da biblioteca para acessar as informações no arquivo e outro método para enviá-las para a API, convertendo o áudio para texto que será consumido pelo modelo de LLM.

## Modelo de LLM

Após a conversão de áudio para texto caso necessário, criamos a seguinte requisição para a API da OpenAI:

```
response = client.chat.completions.create(
            model="gpt-4",
            messages=[
                {
                    "role": "system",
                    "content": (
                        "Você é um assistente de controle de robô. Interprete a entrada do usuário e decida "
                        "quando acionar a função 'navigate_robot'."
                    ),
                },
                {"role": "user", "content": strMsg},
            ],
            functions=[
                {
                    "name": "navigate_robot",
                    "description": "Mova o robô para uma localização específica.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "place_name": {"type": "text", "description": "Nome do ponto de destino."},
                        },
                        "required": ["place_name"],
                    },
                }
            ],
)
```

A requisição acima começa selecionando o tipo e fornecendo um contexto básico para o modelo, após isso, define uma função base para fazer a estruturação da resposta e por fim define o que esperamos no retorno.