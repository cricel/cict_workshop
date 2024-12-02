from langchain_core.messages import HumanMessage
from langchain_google_genai import ChatGoogleGenerativeAI

import os

class MechLMMCore:
    def __init__(self, data_path = "../output"):
        os.environ['GOOGLE_API_KEY'] = 'AIzaSyCFu0QxC6bHASollQK7VZsTGuZx402YVVo'
        
        self.gemini_model = ChatGoogleGenerativeAI(
            model="gemini-1.5-pro",
            temperature=0,
            max_tokens=None,
            timeout=None,
            max_retries=2,
        )
        
        self.mechlmm_model = self.gemini_model

    def chat(self, _question, _tools = None, _base_imgs = None, _schema = None, _tag = None, _model = None):
        print("------ llm chat calling ------")

        return_type = "json"
        lmm_model = None
        content_list = [
            {"type": "text", "text": _question}
        ]

        ## check schema
        if(_schema):
            if (_tools != None):
                return "schema did not work with tool output", _tag, "Error"
            lmm_model = self.mechlmm_model.with_structured_output(_schema)
        else:
            lmm_model = self.mechlmm_model
        
        ## check tools
        if(_tools): 
            # lmm_model = self.mechlmm_model.bind_tools(_tools)
            lmm_model = self.mechlmm_model.bind_tools(_tools, tool_choice="any")

        ## check imgs
        if(_base_imgs):
            for img_url in _base_imgs:
                content_list.append(
                    {
                        "type": "image_url",
                        "image_url": img_url
                    }
                )

        query = [
                HumanMessage(
                    content= content_list
                )
            ]
        
        
        _result = lmm_model.invoke(query)
        
        print(_result)

        try:
            # schema
            return_type = "json"
            return _result[0]["args"], _tag, return_type
        except:
            pass
        
        try:
            # tools
            return_type = "tools"
            if(_result.tool_calls != []):
                return _result.tool_calls, _tag, return_type
        except:
            pass

        try:
            # text
            return_type = "content"
            return _result.content, _tag, return_type
        except:
            pass
            
        try:
            # claude structure output
            return_type = "json"
            return _result, _tag, return_type
        except:
            pass
    
if __name__ == '__main__':
    mechlmm_core = MechLMMCore()
    mechlmm_core.chat("hi")