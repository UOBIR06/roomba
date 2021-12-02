from math import ceil
from typing import Any, Optional


class Heap(object):
    """Implementation of a min-heap."""

    def __init__(self):
        self.array = []

    def push(self, element: Any, priority: float):
        """Add element to heap."""
        self.array.append((element, priority))
        i = len(self.array) - 1

        while True:
            j = int(ceil(i / 2) - 1)
            parent = self.array[j]

            # Swap with parent if lower priority
            if parent[1] > priority:
                self.array[j], self.array[i] = self.array[i], self.array[j]
            else:
                break

    def pop(self) -> Optional[Any]:
        """Remove root element (lowest priority)."""
        if not self.array:
            return None

        root = self.array[0]
        last = self.array.pop()
        if self.array:
            self.array[0] = last

        i = 0
        length = len(self.array)

        while True:
            j = i
            l = 2 * (i + 1)
            r = 2 * (i + 1) + 1

            if l < length and self.array[l][1] < self.array[j][1]:
                j = l
            if r < length and self.array[r][1] < self.array[j][1]:
                j = r

            if j != i:
                self.array[j], self.array[i] = self.array[i], self.array[j]
                i = j
            else:
                break

        return root[0]

    def has(self, element: Any) -> bool:
        return element in [e for e, _ in self.array]

    def __bool__(self):
        return len(self.array) != 0
