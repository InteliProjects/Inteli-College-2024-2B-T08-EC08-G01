import requests
import json
import os

from dotenv import load_dotenv

load_dotenv()

API_KEY = os.getenv('API_KEY')
ASSISTANT_URL = os.getenv('SERVICE_URL')
ASSISTANT_ID = os.getenv('ASSISTANT_ID')

# Function to create a session
def create_session():
    url = f"{ASSISTANT_URL}/v2/assistants/{ASSISTANT_ID}/sessions?version=2024-08-25"
    response = requests.post(url, auth=('apikey', API_KEY))
    response.raise_for_status()
    return response.json()['session_id']

# Function to send a message to the assistant
def send_message(session_id, user_input):
    url = f"{ASSISTANT_URL}/v2/assistants/{ASSISTANT_ID}/sessions/{session_id}/message?version=2023-06-15"
    headers = {'Content-Type': 'application/json'}
    payload = {
        "input": {
            "text": user_input
        }
    }
    response = requests.post(url, json=payload, auth=('apikey', API_KEY), headers=headers)
    response.raise_for_status()
    return response.json()

# Function to trigger an action based on assistant's response
def trigger_action_based_on_response(response):
    intents = response.get('output', {}).get('intents', [])
    if intents:
        intent = intents[0]['intent']  # Assuming the highest confidence intent
        print(f"Detected intent: {intent}")
        if intent == "trigger_action":
            print("Triggering specific action...")
            # Replace with your action code
            execute_my_function()

# Your specific function to execute
def execute_my_function():
    print("Action executed!")

# Main execution
if __name__ == "__main__":
    try:
        session_id = create_session()
        print(f"Session created: {session_id}")
        
        user_prompt = input("Enter your query: ")
        response = send_message(session_id, user_prompt)
        print("Assistant Response:", json.dumps(response, indent=2))
        
        trigger_action_based_on_response(response)
    except Exception as e:
        print(f"An error occurred: {e}")
