import ctypes as C
import math
import unittest
import numpy as np
from branchguard.native import Controller, Observation, Path, DISTANCES, SCAN


def observation(lines):
    obs = Observation()
    for r, s in enumerate(DISTANCES):
        ys = sorted(set(round(fn(s), 6) for fn in lines))
        obs.rows[r].count = len(ys)
        for j, y in enumerate(ys):
            obs.rows[r].points[j].y = y
            obs.rows[r].points[j].quality = 1.
    return obs


class CoreTests(unittest.TestCase):
    def test_blank_and_flat_dark_are_not_paths(self):
        ctl = Controller()
        for value in (0, 25, 170, 255):
            out, obs = ctl.step(np.full((120,188),value,dtype=np.uint8))
            self.assertEqual(sum(row.count for row in obs.rows), 0)
            self.assertEqual(out.status, 0)
            self.assertEqual(out.speed, 0.)

    def test_run_centres_match_reference_projection(self):
        image = np.full((120,188),220,dtype=np.uint8)
        for r in SCAN:
            image[r,40:46] = 25
            image[r,110:116] = 25
        obs = Controller().extract(image)
        for r,s in enumerate(DISTANCES):
            self.assertEqual(obs.rows[r].count, 2)
            self.assertAlmostEqual(obs.rows[r].points[0].y,(94-42.5)*s/80,places=5)
            self.assertAlmostEqual(obs.rows[r].points[1].y,(94-112.5)*s/80,places=5)

    def test_search_keeps_both_fork_branches(self):
        ctl = Controller('spatial')
        obs = observation([lambda s:.5*max(s-.4,0),lambda s:-.5*max(s-.4,0)])
        paths = (Path*8)()
        n = ctl.lib.bg_search(C.byref(obs),paths)
        self.assertGreaterEqual(n,2)
        ends = [ctl.lib.bg_path_at(C.byref(paths[i]),1.15) for i in range(n)]
        self.assertGreater(max(ends),.2)
        self.assertLess(min(ends),-.2)

    def test_left_right_instruction_is_mirror_symmetric(self):
        obs = observation([lambda s:.5*max(s-.4,0),lambda s:-.5*max(s-.4,0)])
        left = Controller('spatial').step_observation(obs,route=1)
        right = Controller('spatial').step_observation(obs,route=-1)
        self.assertGreater(left.steering,0)
        self.assertLess(right.steering,0)
        self.assertAlmostEqual(left.steering,-right.steering,places=4)

    def test_straight_instruction_selects_middle(self):
        obs = observation([lambda s:0,lambda s:.5*max(s-.4,0),lambda s:-.5*max(s-.4,0)])
        out = Controller('spatial').step_observation(obs,route=0)
        self.assertAlmostEqual(out.steering,0,places=5)

    def test_single_frame_replacement_is_deferred_then_confirmed(self):
        ctl = Controller()
        ctl.step_observation(observation([lambda s:0]))
        shifted = observation([lambda s:.10])
        one = ctl.step_observation(shifted)
        self.assertEqual(one.status,2)
        self.assertAlmostEqual(one.steering,0,places=5)
        for _ in range(4):
            accepted = ctl.step_observation(shifted)
        self.assertEqual(accepted.status,1)
        self.assertGreater(accepted.steering,.1)

    def test_shared_near_prefix_does_not_delay_a_real_fork(self):
        ctl=Controller()
        ctl.step_observation(observation([lambda s:0]))
        out=ctl.step_observation(observation([lambda s:1.2*max(s-.60,0)]),route=1)
        self.assertEqual(out.status,1)
        self.assertLess(out.discrepancy,.001)
        far=ctl.lib.bg_path_at(C.byref(out.selected),1.15)
        self.assertGreater(far,.1)

    def test_dropout_expires_and_stop_is_not_track(self):
        ctl = Controller()
        ctl.step_observation(observation([lambda s:.02]))
        self.assertEqual(ctl.step_observation(Observation()).status,3)
        for _ in range(7):
            out = ctl.step_observation(Observation())
        self.assertEqual(out.status,0)
        self.assertEqual(out.speed,0)
        self.assertEqual(ctl.state.has_previous,0)

    def test_odometry_transform_preserves_stationary_world_line(self):
        ctl = Controller()
        ctl.step_observation(observation([lambda s:.12*s]))
        ds, da = .025, .012
        # Analytical body-frame transform of a stationary straight line.
        cs,sn=math.cos(da),math.sin(da)
        slope=(-sn+.12*cs)/(cs+.12*sn)
        intercept=.12*ds*cs - slope*.12*ds*sn
        out = ctl.step_observation(observation([lambda s:slope*s+intercept]),distance=ds,yaw_change=da)
        self.assertLess(out.discrepancy,.0001)
        self.assertEqual(out.status,1)

    def test_invalid_motion_returns_finite_stop(self):
        ctl = Controller()
        obs=observation([lambda s:0])
        for kw in ({'distance':float('nan')},{'dt':0},{'route':7},{'yaw_change':2}):
            out=ctl.step_observation(obs,**kw)
            self.assertEqual(out.status,0)
            self.assertEqual(out.speed,0)
            self.assertTrue(math.isfinite(out.steering))

    def test_invalid_candidate_count_and_nan_cannot_reuse_old_path(self):
        ctl=Controller()
        ctl.step_observation(observation([lambda s:0]))
        obs=observation([lambda s:0])
        obs.rows[4].count=7
        self.assertEqual(ctl.step_observation(obs).status,0)
        obs.rows[4].count=1
        obs.rows[4].points[0].y=float('nan')
        self.assertEqual(ctl.step_observation(obs).status,0)

    def test_two_missing_rows_still_form_observed_path(self):
        obs=observation([lambda s:.04*s])
        obs.rows[5].count=0
        obs.rows[6].count=0
        out=Controller('spatial').step_observation(obs)
        self.assertGreaterEqual(out.selected.support,9)
        self.assertEqual(out.status,1)


if __name__=='__main__':
    unittest.main()
