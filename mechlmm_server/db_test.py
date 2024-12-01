import sqlite3

# Connect to SQLite database (or create it if it doesn't exist)
conn = sqlite3.connect('robot_status.db')

# Create a cursor object
cursor = conn.cursor()

# Create a table
cursor.execute('''
CREATE TABLE IF NOT EXISTS object_info (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    pose TEXT NOT NULL
)
''')

# Insert data into the table
cursor.execute('''
INSERT INTO object_info (name, pose)
VALUES (?, ?)
''', ("apple", "3,4,2,1.4,2.2,3.1"))

# Commit changes and close the connection
conn.commit()
conn.close()

print("Table created and data inserted successfully!")
