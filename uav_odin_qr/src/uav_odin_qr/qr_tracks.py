"""Associate detections by content and image location to name TF children."""

import math
import uuid


def _unambiguous_best(candidates):
    """Pick the closest candidate if there is no competing match nearby."""
    ordered = sorted(candidates)
    if not ordered or (len(ordered) > 1 and ordered[1][0] - ordered[0][0] <= 8.0):
        return None
    return ordered[0][1]


class QrTracks:
    def __init__(self):
        self.prefix = 'uav_odin_qr_' + uuid.uuid4().hex[:12]
        self.next_id = 1
        self.active = {}

    def clear(self):
        self.active.clear()

    def update(self, centers, received):
        previous = {key: track for key, track in self.active.items() if received - track[3] <= 0.7}
        by_observation = [[] for _ in centers]
        by_track = {key: [] for key in previous}
        for index, (content, u, v) in enumerate(centers):
            for key, (old_content, old_u, old_v, _) in previous.items():
                if content != old_content:
                    continue
                distance = math.hypot(u - old_u, v - old_v)
                if distance <= 100.0:
                    by_observation[index].append((distance, key))
                    by_track[key].append((distance, index))

        matches = {}
        for index, candidates in enumerate(by_observation):
            key = _unambiguous_best(candidates)
            if key is not None and _unambiguous_best(by_track[key]) == index:
                matches[index] = key

        children = []
        claimed = set(matches.values())
        current = dict(previous)
        for index, (content, u, v) in enumerate(centers):
            key = matches.get(index)
            if key is None:
                # Retire ambiguous candidates so they cannot reclaim an ID next frame.
                for _, candidate in by_observation[index]:
                    if candidate not in claimed:
                        current.pop(candidate, None)
                key = self.next_id
                self.next_id += 1
            current[key] = (content, u, v, received)
            children.append(f'{self.prefix}_{key}')
        self.active = current
        return children
