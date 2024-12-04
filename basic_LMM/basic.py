import google.generativeai as genai
from setup import API_KEY

# Configure the API key
genai.configure(api_key=API_KEY())

# Define the prompt
prompt = "Describe the benefits of renewable energy."

# Generate text
model = genai.GenerativeModel("gemini-1.5-flash")
response = model.generate_content(prompt)

# Output the generated text
print(response.text)
