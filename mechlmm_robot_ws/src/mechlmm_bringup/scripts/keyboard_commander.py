from termcolor import colored
import requests
import json
from concurrent.futures import ThreadPoolExecutor

class KeyboardCommander:
    def __init__(self):
        self.api_url = "http://0.0.0.0:5001/mechlmm/chat"
        self.api_headers = {
            "Content-Type": "application/json"
        }

        self.command_list = []
        self.executor = ThreadPoolExecutor(max_workers=4)  # Adjust the number of workers as needed

    def send_request(self, _question):
        payload = {
            "question": _question
        }

        def make_request():
            try:
                response = requests.post(self.api_url, headers=self.api_headers, data=json.dumps(payload))
                if response.status_code == 200:
                    print(colored(f"|AI Assistant|: {response.json()['result']}", "green"))
                else:
                    print(colored(f"|AI Assistant|: Failed to get a response. Status code: {response.status_code}", "red"))
                    print(colored(response.text, "red"))
            except requests.RequestException as e:
                print(f"An error occurred: {e}")

        # Submit the request to the ThreadPoolExecutor
        self.executor.submit(make_request)
    
    def command_checker(self, _question):
        query = "dont answer the question, and check if this question belong to a robot command, return 'True' if it does, return 'False' if it doesnt, and nothing else: \n"
        payload = {
            "question": query + _question
        }

        def make_request():
            try:
                response = requests.post(self.api_url, headers=self.api_headers, data=json.dumps(payload))
                if response.status_code == 200:
                    print(colored(f"|AI Assistant Background|: {response.json()['result']}", "green"))
                    if(self.remove_whitespace(response.json()['result']) == "true"):
                        self.command_list.append(_question)
                    elif(self.remove_whitespace(response.json()['result']) == "false"):
                        pass
                    else:
                        print("something went wrong")
                    print(self.command_list)
                else:
                    print(colored(f"|AI Assistant|: Failed to get a response. Status code: {response.status_code}", "red"))
                    print(colored(response.text, "red"))
            except requests.RequestException as e:
                print(f"An error occurred: {e}")

        self.executor.submit(make_request)

    def remove_whitespace(self, _input_string):
        return _input_string.lower().replace(" ", "").replace("\n", "").replace("\t", "")

if __name__ == '__main__':
    keyboard_commander = KeyboardCommander()

    print("Type 'exit' to quit the program.")

    while True:
        user_question = input("User: ")

        if user_question.lower() == "exit":
            print("Exiting the program. Goodbye!")
            break

        keyboard_commander.send_request(user_question)
        keyboard_commander.command_checker(user_question)
