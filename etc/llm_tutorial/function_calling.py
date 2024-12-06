import google.generativeai as genai
from setup import API_KEY

# Configure the API key for the Generative AI model
genai.configure(api_key=API_KEY())

# Define basic arithmetic functions that will be used by the AI model when needed
def add(a: float, b: float):
    """Returns the sum of a and b."""
    return a + b

def subtract(a: float, b: float):
    """Returns the result of a - b."""
    return a - b

def multiply(a: float, b: float):
    """Returns the product of a and b."""
    return a * b

def divide(a: float, b: float):
    """Returns the result of a / b. Assumes b is not zero."""
    return a / b

# Create a Generative AI model instance
model = genai.GenerativeModel(
    model_name="gemini-1.5-flash",  # Specify the Gemini model version
    tools=[add, subtract, multiply, divide]  # Register the arithmetic functions as callable tools
)

# Start a chat session with the model
chat = model.start_chat(enable_automatic_function_calling=True)  # Enable automatic function calling by the model

# Send a query to the model
response = chat.send_message(
    "I have 57 cats, each owns 44 mittens, how many mittens is that in total?"
)

# Print the model's response
print(response.text)
