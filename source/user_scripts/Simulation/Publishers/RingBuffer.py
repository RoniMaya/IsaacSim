# ring_buffer.py
from collections import deque
from threading import Lock
from typing import Optional, Any

class RingBuffer:
    """
    Thread-safe bounded buffer between producers (sensors) and consumers (publishers).

    - 'latest' policy (video): always keep newest N, overwrite/drop oldest.
    - 'keep' policy (detections): block dropping, reject push if full.
    """

    def __init__(self, capacity: int, drop_policy: str = "latest") -> None:
        self._capacity = capacity
        self._policy = drop_policy  # "latest" or "keep"
        self._q = deque(maxlen=capacity)
        self._lock = Lock()

    def push(self, item: Any) -> bool:
        """
        Add an item to the buffer.
        Returns True if accepted, False if dropped (only in 'keep' mode).
        """
        with self._lock:
            if len(self._q) >= self._capacity and self._policy == "keep":
                # refuse to overwrite in keep mode
                return False
            self._q.append(item)  # deque with maxlen auto-drops oldest if needed
            return True

    def pop(self) -> Optional[Any]:
        """
        Remove and return the oldest item (FIFO order).
        Returns None if empty.
        """
        with self._lock:
            if not self._q:
                return None
            return self._q.popleft()

    def latest(self) -> Optional[Any]:
        """
        Return the newest item without removing it.
        Good for video (latest frame wins).
        Returns None if empty.
        """
        with self._lock:
            return self._q[-1] if self._q else None

    def __len__(self) -> int:
        """Current number of items in the buffer."""
        with self._lock:
            return len(self._q)
