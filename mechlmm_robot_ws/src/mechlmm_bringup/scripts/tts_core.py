import asyncio
import aiohttp

async def send_request(session, user_input):
    """
    Send an asynchronous API request with the given user input.
    """
    url = "https://api.example.com/endpoint"  # Replace with your actual API endpoint
    data = {"input": user_input}  # Adjust payload format as required

    try:
        async with session.post(url, json=data) as response:
            result = await response.json()  # Parse JSON response
            print(f"Response for '{user_input}': {result}")
    except Exception as e:
        print(f"Error processing '{user_input}': {e}")

async def main():
    """
    Main asynchronous loop to take user input and send API requests.
    """
    async with aiohttp.ClientSession() as session:
        while True:
            user_input = input("Enter your input (or 'exit' to quit): ")
            if user_input.lower() == 'exit':
                print("Exiting...")
                break
            # Schedule the API request and continue
            asyncio.create_task(send_request(session, user_input))

# Run the asyncio event loop
if __name__ == "__main__":
    asyncio.run(main())
