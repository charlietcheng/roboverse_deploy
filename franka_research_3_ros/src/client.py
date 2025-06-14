import os
import numpy as np
from datetime import datetime
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, List
# import draccus
import requests
from typing import Any, Dict, List
from numpy.lib.format import descr_to_dtype, dtype_to_descr
from base64 import b64decode, b64encode

class ClientConfig:
    # Server Configuration
    # server_ip: str = "0.0.0.0"
    def __init__(self, host="172.17.0.1",  port="21034"):
        self.server_ip: str = host # if access server host inside docker container
        self.server_port: int = port

def get_config(host, port) -> ClientConfig:
        return ClientConfig(host, port)

def numpy_serialize(o):
    if isinstance(o, (np.ndarray, np.generic)):
        data = o.data if o.flags["C_CONTIGUOUS"] else o.tobytes()
        return {
            "__numpy__": b64encode(data).decode(),
            "dtype": dtype_to_descr(o.dtype),
            "shape": o.shape,
        }

    msg = f"Object of type {o.__class__.__name__} is not JSON serializable"
    raise TypeError(msg)


def numpy_deserialize(dct):
    if "__numpy__" in dct:
        np_obj = np.frombuffer(b64decode(dct["__numpy__"]), descr_to_dtype(dct["dtype"]))
        return np_obj.reshape(shape) if (shape := dct["shape"]) else np_obj[0]
    return dct


def convert_numpy_in_dict(data, func):
    """
    Recursively processes a JSON-like dictionary, converting any NumPy arrays
    or lists of NumPy arrays into a serializable format using the provided function.

    Args:
        data: The JSON-like dictionary or object to process.
        func: A function to apply to each NumPy array to make it serializable.

    Returns:
        The processed dictionary or object with all NumPy arrays converted.
    """
    if isinstance(data, dict):
        if "__numpy__" in data:
            return func(data)
        return {key: convert_numpy_in_dict(value, func) for key, value in data.items()}
    elif isinstance(data, list):
        return [convert_numpy_in_dict(item, func) for item in data]
    elif isinstance(data, (np.ndarray, np.generic)):
        return func(data)
    else:
        return data
    
class Message(object):
    def __init__(self):
        pass
    
    def serialize(self):
        raise NotImplementedError
    
    @classmethod
    def deserialize(cls, response: Dict[str, Any]):
        raise NotImplementedError


class RequestMessage(Message):
    def __init__(self, image: np.ndarray, instruction: str, history: List[np.ndarray], dataset_name: str, timestamp: str, gt_action: np.ndarray=None ):
        self.image, self.instruction, self.history, self.dataset_name, self.timestamp = image, instruction, history, dataset_name, timestamp
        self.gt_action = gt_action if gt_action is not None else []
    
    def __str__(self):
        string = f"""
            Image: {self.image.shape} {self.image.dtype}
            Instruction: {self.instruction}
            History: {len(self.history)}
            Dataset Name: {self.dataset_name}
            Timestamp: {self.timestamp}
            Ground Truth: {self.gt_action}
            """
        return string

    def serialize(self):
        msg = {
            "image": self.image,
            "instruction": self.instruction,
            "history": self.history,
            "dataset_name": self.dataset_name,
            "timestamp": self.timestamp,
            "gt_action": self.gt_action
        }
        return convert_numpy_in_dict(msg, numpy_serialize)
    
    @classmethod
    def deserialize(cls, response: Dict[str, Any]):
        response = convert_numpy_in_dict(response, numpy_deserialize)
        return cls(
            image=response["image"],
            instruction=response["instruction"],
            history=response["history"],
            dataset_name=response["dataset_name"],
            timestamp=response["timestamp"],
            gt_action=response["gt_action"]
        )


class ResponseMessage(Message):
    def __init__(self, action: np.ndarray, token_accuracy: float):
        self.action = action
        self.token_accuracy = token_accuracy
    
    def serialize(self):
        msg = {
            "action": self.action,
            "token_accuracy": self.token_accuracy
        }
        return convert_numpy_in_dict(msg, numpy_serialize)
    
    @classmethod
    def deserialize(cls, response: Dict[str, Any]):
        response = convert_numpy_in_dict(response, numpy_deserialize)
        return cls(action=response["action"], token_accuracy=response["token_accuracy"])


class HttpPolicyClient(object):
    def __init__(self, server_ip: str, server_port: int):
        self.server_ip, self.server_port = server_ip, server_port
    
    @property
    def timestamp(self):
        return str(datetime.now()).replace(" ", "_").replace(":", "-")
    
    def query_action(self, image: np.ndarray, instruction: str, 
                     history: Optional[List[np.ndarray]] = None, 
                     dataset="grasp",
                     gt_action=None) -> np.ndarray:
        if history is None:
            history = []
        request = RequestMessage(image, instruction, history, dataset, self.timestamp, gt_action)
        response = requests.post(
            f"http://{self.server_ip}:{self.server_port}/act",
            json=request.serialize()
        )
        response = ResponseMessage.deserialize(response.json())
        if response.token_accuracy is None:
            return response.action
        return response.action, response.token_accuracy

if __name__ == "__main__":
    # class FakeArg:
    #     port:str = 21034
    
    # args = FakeArg()
    # cfg = get_config(host="172.17.0.1", port=26784)
    cfg = get_config(host="127.0.0.1", port=26784)
    client = HttpPolicyClient(cfg.server_ip, cfg.server_port)
    
    from PIL import Image
    import os
    import cv2
    
    # obs = np.array(Image.open("steore-left.png"), dtype=np.uint8)
    obs = np.array(np.zeros((360, 640, 3), dtype=np.uint8))
    # obs_req = np.array(Image.open("req.png"), dtype=np.uint8)

    # obs = np.array(Image.open("req-loaded.png"), dtype=np.uint8)
    # Image.fromarray(np.abs(obs-obs_req)).save("diff.png")

    # obs = np.array(Image.open("loaded.png"), dtype=np.uint8)
    # obs = cv2.resize(obs, (256, 256))
    instruction = "Pick up head shoulders supreme."
    # gt_action = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    gt_action = None
    action = client.query_action(obs, instruction, gt_action=gt_action) # delta: xyz, rpy, openness
    print("denormalized action: ", action)

    # action_req = client.query_action(obs_req, instruction, gt_action=gt_action) # delta: xyz, rpy, openness

    # print("diff-xyz", np.linalg.norm(np.array(action)[:3] - np.array(action_req)[:3]) )
    # print("diff-rpy", np.rad2deg((np.array(action)[3:] - np.array(action_req)[3:])) )