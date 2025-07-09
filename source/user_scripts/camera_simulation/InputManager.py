import threading



class InputManager():
    def __init__(self):
        """
        Initializes the InputManager with a thread lock and an empty set of keys.
        """

        self._lock = threading.Lock()
        self._keys = set()


    def press_key(self,key):
        """
        Registers a key as pressed by adding it to the set of active keys.
        Args:
            key (str): The key to register as pressed.
        """

        key = key.upper()
        with self._lock:
            self._keys.add(key)
    
    def release_key(self,key):
        """
        Release a pressed key by removing it from the active keys set.
        Args:
            key (str): The key to release.
        """

        key = key.upper()
        with self._lock:
            if key in self._keys:
                self._keys.discard(key)
        

    def get_pressed_keys(self):
        """
        Return a copy of the currently pressed keys.
        Returns:
            list: A copy of the list of pressed keys.
        """

        with self._lock:
            return self._keys.copy()