import os
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_core.runnables.history import RunnableWithMessageHistory
from langchain_community.chat_message_histories import ChatMessageHistory

from setup import API_KEY
# Set up Google API key
os.environ["GOOGLE_API_KEY"] = API_KEY()

# Initialize Gemini model
llm = ChatGoogleGenerativeAI(model="gemini-1.5-pro")

# Create prompt template with memory placeholder
prompt = ChatPromptTemplate.from_messages([
    MessagesPlaceholder(variable_name="chat_history"),
    ("human", "{input}")
])

# Construct the chain
chain = prompt | llm

# Create message history for persistent memory
def get_session_history(session_id: str):
    if session_id not in store:
        store[session_id] = ChatMessageHistory()
    return store[session_id]

# Initialize session store
store = {}

# Wrap chain with message history
memory_chain = RunnableWithMessageHistory(
    chain,
    get_session_history,
    input_messages_key="input",
    history_messages_key="chat_history"
)

# Function to interact with AI
def chat_with_ai(input_text, session_id="default_session"):
    result = memory_chain.invoke(
        {"input": input_text},
        config={"configurable": {"session_id": session_id}}
    )
    return result.content

# Example usage
def main():
    # First interaction
    print(chat_with_ai("My name is Alex and I'm a software engineer.", "session1"))
    
    # Second interaction - should remember previous context
    print(chat_with_ai("What was my profession?", "session1"))
    
    # Different session won't remember previous context
    print(chat_with_ai("What was my name?", "session2"))

if __name__ == "__main__":
    main()