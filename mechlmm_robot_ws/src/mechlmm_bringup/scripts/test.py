import json

# Data as a Python dictionary
data = {
    "position": {
        "x": 0.49173858761787415,
        "y": 0.6525877118110657,
        "z": 0.0
    },
    "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": -0.026288548254380737,
        "w": 0.9996543963944124
    }
}

# Convert dictionary to JSON string
json_string = json.dumps(data, indent=2)

print(json_string)
