from termcolor import colored
import requests
import json

url = "http://0.0.0.0:5001/mechlmm/chat"
headers = {
    "Content-Type": "application/json"
}

def send_request(question):
    payload = {
        "question": question
    }

    try:
        response = requests.post(url, headers=headers, data=json.dumps(payload))
        if response.status_code == 200:
            print(colored(f"|AI Assistant|: {response.json()['result']}", "green"))
        else:
            print(colored(f"|AI Assistant|: Failed to get a response. Status code: {response.status_code}", "red"))
            print(colored(response.text, "red"))
    except requests.RequestException as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    print("Type 'exit' to quit the program.")

    while True:
        user_question = input("User: ")

        if user_question.lower() == "exit":
            print("Exiting the program. Goodbye!")
            break

        send_request(user_question)
