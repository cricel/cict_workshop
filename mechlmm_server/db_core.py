import sqlite3
import json

class DBCore:
    def __init__(self):
        self.obj_info_table_name = "object_info"
        self.database = 'robot_status.db'

    def get_db_connection(self):
        """
        Create a new SQLite connection for each request.
        """
        connection = sqlite3.connect(self.database)
        connection.row_factory = sqlite3.Row
        return connection

    def dummy_data_injection(self):
        with self.get_db_connection() as conn:
            cursor = conn.cursor()

            cursor.execute(f"DROP TABLE IF EXISTS {self.obj_info_table_name}")
            conn.commit()

            cursor.execute(f'''
                CREATE TABLE IF NOT EXISTS {self.obj_info_table_name} (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT NOT NULL,
                    pose TEXT NOT NULL
                )
                '''
            )

            data = [
                ("stop sign", {
                    "position": {"x": 0.49173858761787415, "y": 0.6525877118110657, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1}
                }),
                ("energy drink", {
                    "position": {"x": -0.7674800157546997, "y": -1.966954231262207, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1}
                }),
                ("drone", {
                    "position": {"x": -0.868584930896759, "y": -0.8731806874275208, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1}
                })
            ]

            for name, pose in data:
                cursor.execute('''
                    INSERT INTO object_info (name, pose)
                    VALUES (?, ?)
                ''', (name, json.dumps(pose)))

            conn.commit()
            print("Table created and data inserted successfully!")

    def get_pose_by_name(self, _name):
        """
        Retrieve pose data for a given name.
        """
        try:
            with self.get_db_connection() as conn:
                cursor = conn.cursor()
                cursor.execute(f"SELECT pose FROM {self.obj_info_table_name} WHERE name = ?", (_name,))
                result = cursor.fetchone()

                if result:
                    return result["pose"]
                else:
                    return None
        except sqlite3.Error as e:
            print(f"An error occurred: {e}")
            return None


if __name__ == '__main__':
    db_core = DBCore()
    db_core.dummy_data_injection()
    print(db_core.get_pose_by_name("stop sign"))
