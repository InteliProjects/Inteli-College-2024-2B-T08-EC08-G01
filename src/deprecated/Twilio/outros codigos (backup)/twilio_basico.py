from flask import Flask, request
from twilio.twiml.messaging_response import MessagingResponse
from openai import OpenAI
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Instantiate OpenAI client
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Initialize Flask
app = Flask(__name__)

@app.route('/bot', methods=['POST'])
def bot():
    incoming_msg = request.values.get('Body', '').lower()
    resp = MessagingResponse()
    msg = resp.message()

    try:
        # Use the new OpenAI client
        response = client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "Você é um assistente virtual responsivo."},
                {"role": "user", "content": incoming_msg},
            ],
        )
        # Extract response text
        chat_response = response.choices[0].message.content
        msg.body(chat_response)

    except Exception as e:
        # Handle errors
        msg.body(f"Erro ao processar sua solicitação: {str(e)}")

    return str(resp)

if __name__ == '__main__':
    app.run()