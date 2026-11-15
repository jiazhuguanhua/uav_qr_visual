import unittest

from uav_odin_qr.qr_tracks import QrTracks


class QrTracksTest(unittest.TestCase):
    def test_same_content_keeps_separate_children_when_order_changes(self):
        tracks = QrTracks()
        left, right = tracks.update([('SAME', 120, 200), ('SAME', 185, 200)], 1.0)
        self.assertNotEqual(left, right)
        self.assertEqual(
            tracks.update([('SAME', 182, 201), ('SAME', 123, 199)], 1.1), [right, left])

    def test_ambiguous_match_and_expired_target_get_new_children(self):
        tracks = QrTracks()
        first = tracks.update([('SAME', 120, 200), ('SAME', 160, 200)], 1.0)
        ambiguous, = tracks.update([('SAME', 140, 200)], 1.1)
        self.assertNotIn(ambiguous, first)
        self.assertEqual(tracks.update([], 2.0), [])
        new, = tracks.update([('SAME', 140, 200)], 2.1)
        self.assertNotIn(new, first + [ambiguous])

    def test_clear_and_restart_do_not_reuse_children(self):
        tracks = QrTracks()
        center = [('A', 20, 20)]
        old, = tracks.update(center, 1.0)
        tracks.clear()
        self.assertNotEqual(tracks.update(center, 2.0)[0], old)
        self.assertNotEqual(QrTracks().update(center, 1.0)[0], old)


if __name__ == '__main__':
    unittest.main()
