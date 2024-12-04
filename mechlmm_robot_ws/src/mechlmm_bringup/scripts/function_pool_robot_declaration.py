from pydantic import BaseModel, Field

class manipulation(BaseModel):
    """Manipulate the target object"""

    target_name: str = Field(..., description="the name of the target object")

class move_robot(BaseModel):
    """move robot to the target direction for certain distance"""

    direction: str = Field(..., description="the direction of robot is moving, the value can only be: forward, backward, turn_left, turn_right")

class idle(BaseModel):
    """Do nothing, as idle"""