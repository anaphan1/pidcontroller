import unittest
from simplepidcontroller import PID

class Process:
    def __init__(self, k=1, tau=1):
        self.k = k
        self.tau = tau
        self.y = 0
    def update(self, u, dt):
        self.y += (self.k * (u - self.y) / self.tau) * dt
        return self.y

class TestPIDController(unittest.TestCase):
    def test_valid_parallel_constants(self):
        pid = PID(2)
        constants = 2, 0.5, 0.5
        pid.parallel_constants = constants
        self.assertEqual(constants, pid.parallel_constants)

    def test_invalid_parallel_constants(self):
        pid = PID(2)
        constants = 2, 0.5, 0.5, 3
        def set_constants():
            pid.parallel_constants = constants
        self.assertRaises(ValueError, set_constants)

    def test_valid_ideal_constants(self):
        pid = PID(2)
        constants = 2, 0.5, 0.5
        pid.ideal_constants = constants
        self.assertEqual(constants, pid.ideal_constants)

    def test_invalid_ideal_constants(self):
        pid = PID(2)
        constants = 2, 0.5, 0.5, 3
        def set_constants():
            pid.parallel_constants = constants
        self.assertRaises(ValueError, set_constants)

    def test_parallel_ideal_conversion(self):
        pid = PID(2)
        parallel_constants = 2, 0.5, 0.2
        ideal_constants = 2, 4, 0.1
        pid.parallel_constants = parallel_constants
        self.assertEqual(ideal_constants, pid.ideal_constants)

    def test_ideal_parallel_conversion(self):
        pid = PID(2)
        ideal_constants = 4, 2, 1.1
        parallel_constants = 4, 2, 4.4
        pid.ideal_constants = ideal_constants
        self.assertEqual(parallel_constants, pid.parallel_constants)

    def test_time_validation(self):
        pid = PID(100)
        pid.ideal_constants = 2, 0, 0
        def compute_u():
            pid.compute(50, 0)
        self.assertRaises(ValueError, compute_u)

    def test_p_control(self):
        pid = PID(100)
        pid.ideal_constants = 2, 0, 0
        self.assertEqual(100, pid.compute(50, 2))

    def test_pi_control(self):
        pid = PID(100)
        process = Process()
        pid.ideal_constants = 2, 0.5, 0

        y = process.update(0, 0.1)

        for _ in range(150):
            u = pid.compute(y, 0.1)
            y = process.update(u, 0.1)
        self.assertAlmostEqual(100, y)

    def test_pid_control(self):
        pid = PID(100)
        process = Process()
        pid.ideal_constants = 2, 0.5, 0.1

        y = process.update(0, 0.1)

        for _ in range(150):
            u = pid.compute(y, 0.1)
            y = process.update(u, 0.1)

        self.assertAlmostEqual(100, y)

    def test_saturation_limits(self):
        pid = PID(100, u_min=0, u_max=200)
        pid.ideal_constants = 2, 0.5, 0
        u = pid.compute(90, 10, False, True)
        self.assertLessEqual(u, pid.U_MAX)
        self.assertGreaterEqual(u, pid.U_MIN)


if __name__ == '__main__':
    unittest.main(verbosity=2)
