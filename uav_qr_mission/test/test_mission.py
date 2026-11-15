import unittest
from types import SimpleNamespace

from uav_qr_mission.mission import Mission


def detection(content, center=100.0, valid=True):
    return SimpleNamespace(content=content, center_u=center, center_v=80.0, depth_valid=valid)


def target(content, center=100.0):
    return SimpleNamespace(content=content, center_u=center, center_v=80.0, transform=object())


class MissionTest(unittest.TestCase):
    def setUp(self):
        self.mission = Mission()
        self.assertEqual(self.mission.set_task('run_01', ['A', 'B', 'C'], 'B'), (True, 'ACCEPTED'))

    def test_find_without_depth_and_match_only_exact_target(self):
        self.mission.observe([detection('A', valid=False), detection('A'), detection('D')])
        self.assertEqual(self.mission.found, {'A'})
        choice = target('B')
        self.assertIs(self.mission.select_transform([detection('B')], [target('A'), choice]), choice.transform)
        self.assertIsNone(self.mission.select_transform([detection('B', 200.0)], [choice]))

    def test_two_same_codes_stay_ambiguous_when_only_one_has_depth(self):
        detections = [detection('B'), detection('B', 200.0, valid=False)]
        self.assertIsNone(self.mission.select_transform(detections, [target('B')]))
        self.assertIsNone(self.mission.select_transform([detections[0]], [target('B'), target('B')]))

    def test_retries_and_new_task(self):
        self.mission.observe([detection('B')])
        self.assertEqual(self.mission.set_task('run_01', ['A', 'B', 'C'], 'B'), (True, 'ALREADY_ACTIVE'))
        self.assertEqual(self.mission.found, {'B'})
        self.assertEqual(self.mission.set_task('run_01', ['A', 'B', 'C'], 'C'), (False, 'MISSION_ID_CONFLICT'))
        self.assertEqual(self.mission.set_task('run_02', ['A', 'A', 'C'], 'A'), (False, 'INVALID_TASK'))
        self.assertEqual(self.mission.set_task('run_02', ['A', 'C', 'D'], 'D'), (True, 'ACCEPTED'))
        self.assertEqual(self.mission.found, set())
        self.assertEqual(self.mission.set_task('run_01', ['A', 'B', 'C'], 'B'), (False, 'MISSION_ID_REUSED'))


if __name__ == '__main__':
    unittest.main()
