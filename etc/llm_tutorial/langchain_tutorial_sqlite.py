import os
import sqlite3
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_core.prompts import ChatPromptTemplate
from setup import API_KEY
# Step 1: Configure SQLite Database
def setup_database():
    conn = sqlite3.connect("langchain_gemini.db")
    cursor = conn.cursor()
    # Create a table to store chat history
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS chat_history (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        user_message TEXT,
        ai_response TEXT
    )
    """)
    conn.commit()
    return conn, cursor

# Step 2: Configure LangChain with Gemini
def setup_gemini():
    # Set up Google Gemini API Key
    os.environ["GOOGLE_API_KEY"] = API_KEY()  # Replace with your actual API key

    # Configure Gemini model
    llm = ChatGoogleGenerativeAI(
        model="gemini-1.5-pro",
        temperature=0.7,
        max_tokens=200
    )
    return llm

# Step 3: Create a LangChain Prompt Template
def setup_prompt():
    # Define the prompt template
    prompt = ChatPromptTemplate.from_messages([
        ("system", "You are a helpful assistant that can answer user questions and also search through chat history."),
        ("human", "{input}"),
    ])
    return prompt

# Step 4: Store Interaction in SQLite
def store_interaction(cursor, user_message, ai_response):
    cursor.execute("INSERT INTO chat_history (user_message, ai_response) VALUES (?, ?)",
                   (user_message, ai_response))
    cursor.connection.commit()

# Step 5: Fetch Chat History from SQLite
def fetch_interaction(cursor, search_query=None):
    if search_query:
        # Search for specific content in the chat history
        cursor.execute("""
        SELECT * FROM chat_history
        WHERE user_message LIKE ? OR ai_response LIKE ?
        """, (f"%{search_query}%", f"%{search_query}%"))
    else:
        # Fetch all chat history
        cursor.execute("SELECT * FROM chat_history")
    return cursor.fetchall()

# Step 6: Chat with AI using LangChain Prompt
def chat_with_ai(llm, prompt, cursor, user_message):
    # Search history if the user wants
    if user_message.lower().startswith("search:"):
        search_query = user_message[len("search:"):].strip()
        results = fetch_interaction(cursor, search_query)
        if not results:
            return "No results found for your query."
        # Format the results as part of the response
        history_results = "\n".join([f"User: {row[1]}, AI: {row[2]}" for row in results])
        return f"Search Results:\n{history_results}"

    # Use LangChain prompt template for structured input
    chain = prompt | llm
    ai_response = chain.invoke({"input": user_message}).content
    # Store interaction in SQLite
    store_interaction(cursor, user_message, ai_response)
    return ai_response

# Step 7: Chat Interface
def chat(llm, prompt, cursor):
    print("Start chatting with Gemini! Type 'exit' to end or 'search:<query>' to search history.")
    while True:
        user_message = input("You: ")
        if user_message.lower() == "exit":
            print("Goodbye!")
            break
        ai_response = chat_with_ai(llm, prompt, cursor, user_message)
        print(f"AI: {ai_response}")

# Step 8: View Chat History
def view_chat_history(cursor):
    interactions = fetch_interaction(cursor)
    for id, user_message, ai_response in interactions:
        print(f"ID: {id}")
        print(f"User: {user_message}")
        print(f"AI: {ai_response}")
        print("-" * 30)

# Main Execution
if __name__ == "__main__":
    # Set up database and model
    conn, cursor = setup_database()
    llm = setup_gemini()
    prompt = setup_prompt()

    # Main menu
    while True:
        print("\n1. Chat with Gemini")
        print("2. View Chat History")
        print("3. Exit")
        choice = input("Enter your choice: ")
        
        if choice == "1":
            chat(llm, prompt, cursor)
        elif choice == "2":
            view_chat_history(cursor)
        elif choice == "3":
            print("Exiting...")
            conn.close()
            break
        else:
            print("Invalid choice. Please select 1, 2, or 3.")
