
class GlobalConfig:

    _instance = None

    use_websocket = False
    websocket_host = '127.0.0.1'
    websocket_port = 9090
    websocket_timeout = 5.0

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    

