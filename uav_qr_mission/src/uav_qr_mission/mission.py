"""Task validation and exact-frame QR matching."""


class Mission:
    def __init__(self):
        self.id = ''
        self.targets = ()
        self.position_target = ''
        self.found = set()
        self.used_ids = set()
        self.capture_only = False

    def set_task(self, mission_id, target_ids, position_target_id, capture_only=False):
        targets = tuple(target_ids)
        if (not mission_id.strip() or len(targets) != 3 or
                not all(target.strip() for target in targets) or len(set(targets)) != 3 or
                position_target_id not in targets):
            return False, 'INVALID_TASK'
        if mission_id == self.id:
            if (targets == self.targets and position_target_id == self.position_target and
                    capture_only == self.capture_only):
                return True, 'ALREADY_ACTIVE'
            return False, 'MISSION_ID_CONFLICT'
        if mission_id in self.used_ids:
            return False, 'MISSION_ID_REUSED'
        self.id, self.targets, self.position_target = mission_id, targets, position_target_id
        self.capture_only = capture_only
        self.found.clear()
        self.used_ids.add(mission_id)
        return True, 'ACCEPTED'

    def observe(self, detections):
        self.found.update(detection.content for detection in detections if detection.content in self.targets)

    def position_detections(self, detections):
        return [detection for detection in detections if detection.content == self.position_target]

    def select_transform(self, detections, targets, content=None):
        content = self.position_target if content is None else content
        matches = [d for d in detections if d.content == content]
        if len(matches) != 1 or not matches[0].depth_valid:
            return None
        detection = matches[0]
        candidates = [target.transform for target in targets
                      if target.content == detection.content and
                      target.center_u == detection.center_u and target.center_v == detection.center_v]
        return candidates[0] if len(candidates) == 1 else None
