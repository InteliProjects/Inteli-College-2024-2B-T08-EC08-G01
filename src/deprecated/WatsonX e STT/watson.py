import json
from fastapi import FastAPI
from ibm_watson import AssistantV2
from ibm_cloud_sdk_core.authenticators import IAMAuthenticator

authenticator = IAMAuthenticator('2rO-QP7YijbqZOyJ5713mDRGhNj5Io1jL4C2TX97Oxlv')
assistant = AssistantV2(
    version='2024-08-25',
    authenticator=authenticator
)

assistant.set_service_url('https://api.us-south.assistant.watson.cloud.ibm.com')


def session():
    ses = assistant.create_session(assistant_id="a5bb7b98-c92c-4627-9518-d81aa65ff44f").get_result()
    session_id = ses.get('session_id')
    return session_id

app = FastAPI()

@app.get("/send")
async def send():
    response = assistant.message(assistant_id="a5bb7b98-c92c-4627-9518-d81aa65ff44f",session_id=session(),input={'message_type': 'text', 'text':"Oi, tudo bem com você?"}).get_result()
    return json.dumps(response)
