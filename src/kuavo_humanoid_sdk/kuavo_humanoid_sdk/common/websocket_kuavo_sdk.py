import roslibpy
from kuavo_humanoid_sdk.common.global_config import GlobalConfig

class WebSocketKuavoSDK:

    _instance = None
    _initialized = False

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if not self._initialized:
            self._initialized = True
            self.client = roslibpy.Ros(host=GlobalConfig.websocket_host, port=GlobalConfig.websocket_port)
            self.client.run(timeout=GlobalConfig.websocket_timeout)
            

    def __del__(self):
        self.client.terminate()
        self.instance = None
