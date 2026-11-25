"""Pure numerical tests. Importing verify_home does not start ROS or devices."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

_spec=importlib.util.spec_from_file_location(
    '_gravity_home_plant_under_test',Path(__file__).with_name('verify_home.py'))
_plant_module=importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_plant_module)
solve_coupled_friction=_plant_module.solve_coupled_friction


class CoupledFrictionTest(unittest.TestCase):
    def test_sticking_axis_reacts_to_other_joint_acceleration(self):
        mass=np.array([[2.,1.],[1.,2.]])
        a,v,f,held=solve_coupled_friction(mass,[0.,1.],[0.,.1],[.75,.2],[.1,.1])
        np.testing.assert_array_equal(held,[True,False])
        np.testing.assert_allclose(a,[0.,.45],atol=1e-12)
        np.testing.assert_allclose(f,[.45,-.1],atol=1e-12)
        np.testing.assert_allclose(mass@a,[0.,1.]+f,atol=1e-12)

    def test_releases_axis_when_coupled_static_reaction_exceeds_bound(self):
        mass=np.array([[2.,1.],[1.,2.]])
        a,v,f,held=solve_coupled_friction(mass,[0.,1.],[0.,.1],[.2,.2],[.1,.1])
        self.assertFalse(held.any())
        np.testing.assert_allclose(f,[.1,-.1],atol=1e-12)
        np.testing.assert_allclose(a,np.linalg.solve(mass,[.1,.9]),atol=1e-12)
        self.assertLess(a[0],0.)

    def test_all_axes_can_hold_within_static_friction(self):
        mass=np.array([[2.,1.],[1.,2.]])
        a,v,f,held=solve_coupled_friction(mass,[.1,-.2],[.001,-.002],[.2,.3],[.1,.1])
        self.assertTrue(held.all())
        np.testing.assert_array_equal(a,[0.,0.])
        np.testing.assert_array_equal(v,[0.,0.])
        np.testing.assert_allclose(f,[-.1,.2],atol=1e-12)

    def test_mass_projection_removes_energy_and_preserves_free_momentum(self):
        mass=np.array([[2.,1.],[1.,2.]])
        original=np.array([.004,.1])
        a,v,f,held=solve_coupled_friction(mass,[0.,1.],original,[.75,.2],[.1,.1])
        self.assertTrue(held[0]);self.assertFalse(held[1])
        self.assertEqual(v[0],0.)
        self.assertAlmostEqual((mass@v)[1],(mass@original)[1])
        self.assertLess(v@mass@v,original@mass@original)

    def test_free_joints_have_coulomb_torque_opposing_velocity(self):
        mass=np.array([[2.,.5],[.5,1.]])
        a,v,f,held=solve_coupled_friction(mass,[.4,-.3],[.1,-.2],[.2,.2],[.03,.04])
        self.assertFalse(held.any())
        np.testing.assert_allclose(f,[-.03,.04])
        np.testing.assert_allclose(mass@a,[.4,-.3]+f)
        self.assertLess(f@v,0.)

    def test_random_coupled_plants_obey_reaction_and_energy_constraints(self):
        rng=np.random.default_rng(20261125)
        for _ in range(300):
            factor=rng.normal(size=(6,6));mass=factor@factor.T+.05*np.eye(6)
            drive=rng.uniform(-.5,.5,6);original=rng.uniform(-.2,.2,6)
            original[rng.random(6)<.5]=0.
            # Include both exactly stationary and low nonzero capture candidates.
            original[rng.integers(0,6)]=.002
            static=rng.uniform(.05,.5,6);sliding=.4*static
            a,v,f,held=solve_coupled_friction(mass,drive,original,static,sliding)
            np.testing.assert_allclose(mass@a,drive+f,atol=1e-10)
            np.testing.assert_array_equal(a[held],np.zeros(np.count_nonzero(held)))
            np.testing.assert_array_equal(v[held],np.zeros(np.count_nonzero(held)))
            self.assertTrue((abs(f[held])<=static[held]+1e-10).all())
            self.assertLessEqual(f@v,1e-10)
            self.assertLessEqual(v@mass@v,original@mass@original+1e-10)
            np.testing.assert_allclose((mass@v)[~held],(mass@original)[~held],atol=1e-10)

    def test_invalid_inertia_or_friction_rejected(self):
        for mass,static,sliding in [
            (np.array([[1.,2.],[2.,1.]]),[.2,.2],[.1,.1]),
            (np.eye(2),[.1,.1],[.2,.1]),
            (np.eye(2),[-.1,.1],[.0,.0]),
        ]:
            with self.assertRaises((ValueError,np.linalg.LinAlgError)):
                solve_coupled_friction(mass,[0.,0.],[0.,0.],static,sliding)


if __name__=='__main__':
    unittest.main()
