import cv2
import numpy as np
import google.generativeai as genai
from PIL import Image
import re

from setup import API_KEY

# Configure the API key for the generative AI model
genai.configure(api_key=API_KEY())

# Create a Generative AI model instance for object detection
model = genai.GenerativeModel(model_name='gemini-1.5-pro')

# Load the input image for processing
input_image = "basic_LMM/dog_bike_car.jpg"
img = Image.open(input_image)  # Open the image using PIL

#bad Prompt
# response = model.generate_content([
#     img,  # Pass the image as input
#     ("Return bounding boxes for all objects in the image in the following format as")
# ])

# Generate content with an object detection prompt
response = model.generate_content([
    img,  # Pass the image as input
    ("Return bounding boxes for all objects in the image in the following format as"
     " a list. \n [ymin, xmin, ymax, xmax, object_name]. If there are more than one object, return separate lists for each object")
])


result = response.text  # Extract the text output containing bounding box data
print(result)
def parse_bounding_box(response):
    """
    Parse the bounding box response from the AI model.
    """
    # Extract bounding box information using regex
    bounding_boxes = re.findall(r'\[(\d+,\s*\d+,\s*\d+,\s*\d+,\s*[\w\s]+)\]', response)
    
    # Convert extracted strings into structured data
    parsed_boxes = []
    for box in bounding_boxes:
        parts = box.split(',')  # Split the components of the bounding box
        numbers = list(map(int, parts[:-1]))  # Convert coordinates to integers
        label = parts[-1].strip()  # Extract and clean the object label
        parsed_boxes.append((numbers, label))  # Append coordinates and label as a tuple
    
    return parsed_boxes

# Parse bounding box data returned by the model
bounding_box = parse_bounding_box(result)

# Dictionary to store unique colors for each object label
label_colors = {}

def draw_bounding_boxes(image, bounding_boxes_with_labels):
    """
    Draw bounding boxes on the image with labels.
    """
    # Ensure the image is in RGB mode for processing
    if image.mode != 'RGB':
        image = image.convert('RGB')
    
    # Convert PIL image to NumPy array for OpenCV compatibility
    img_array = np.array(image)
    
    # Get dimensions of the image
    width, height = img_array.shape[1], img_array.shape[0]
    
    # Iterate through bounding boxes and draw them
    for bounding_box, label in bounding_boxes_with_labels:
        # Extract and normalize bounding box coordinates
        ymin, xmin, ymax, xmax = bounding_box
        x1 = int(xmin / 1000 * width)  # Scale coordinates to image size
        y1 = int(ymin / 1000 * height)
        x2 = int(xmax / 1000 * width)
        y2 = int(ymax / 1000 * height)
        
        # Assign a unique color to each label
        if label not in label_colors:
            color = tuple(np.random.randint(0, 256, (3,)).tolist())  # Generate a random color
            label_colors[label] = color
        else:
            color = label_colors[label]
        
        # OpenCV drawing parameters
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_thickness = 1
        box_thickness = 2
        
        # Calculate text size for the label background
        text_size = cv2.getTextSize(label, font, font_scale, font_thickness)[0]
        
        # Draw a filled rectangle as a label background
        text_bg_x1 = x1
        text_bg_y1 = y1 - text_size[1] - 5
        text_bg_x2 = x1 + text_size[0] + 8
        text_bg_y2 = y1
        cv2.rectangle(img_array, (text_bg_x1, text_bg_y1), (text_bg_x2, text_bg_y2), color, -1)
        
        # Add label text over the rectangle
        cv2.putText(img_array, label, (x1 + 2, y1 - 5), 
                    font, font_scale, (255, 255, 255), font_thickness)
        
        # Draw the bounding box
        cv2.rectangle(img_array, (x1, y1), (x2, y2), color, box_thickness)
    
    # Convert back to PIL Image for saving/displaying
    output_image = Image.fromarray(img_array)
    return output_image

# Draw bounding boxes on the image
output = draw_bounding_boxes(img, bounding_box)

# Save or display the final output image
output.save('output_image.jpg')  # Save the output image
cv2.imshow('Object Detection', cv2.cvtColor(np.array(output), cv2.COLOR_RGB2BGR))  # Display the image
cv2.waitKey(0)  # Wait for a key press to close the display window
cv2.destroyAllWindows()  # Clean up OpenCV windows
