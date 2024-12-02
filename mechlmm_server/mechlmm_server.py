from mechlmm_core import MechLMMCore

from flask import Flask, request, jsonify, render_template

app = Flask(__name__)

mechlmm_core = MechLMMCore()

@app.route('/')
def root():
    return render_template('index.html')

@app.route('/mechlmm/', methods=['GET'])
def mechlmm_api():
    return jsonify({'content': 'welcome to mechlmm'})   

@app.route('/mechlmm/chat', methods=['POST'])
def chat():
    data = request.json
    
    if not data or 'question' not in data:
        return jsonify({'error': 'Invalid Data'}), 400
    
    question = data['question']
    schema = data.get('schema', None)
    tag = data.get('tag', None)
    base_img = data.get('base_img', None)
    tools = data.get('tools', None)
    model = data.get('model', None)

    result, return_tag, result_type = mechlmm_core.chat(question, tools, base_img, schema, tag, model)

    return jsonify({"result": result, 
                    "tag": return_tag,
                    "type": result_type
                    })

def main():
    app.run(host='0.0.0.0', port=5001)

if __name__ == '__main__':
    main()


# curl -X POST "0.0.0.0:5001/mechlmm/chat" -H "Content-Type: application/json" -d '{"question":"hi"}'
