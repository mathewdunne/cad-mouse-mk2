import unittest

import numpy as np

import calibrate_motion as cal


class CalibrateMotionTests(unittest.TestCase):
    def test_parse_raw_line(self):
        sample = cal.parse_raw_line("CALRAW,123,1,2,3,4,5,6,7,8,9")
        self.assertEqual(sample.millis, 123)
        np.testing.assert_allclose(sample.values, np.arange(1, 10, dtype=float))

        with self.assertRaises(ValueError):
            cal.parse_raw_line(">X:1.0")

    def test_synthetic_calibration_recovers_known_matrix(self):
        rng = np.random.default_rng(1234)
        basis, _ = np.linalg.qr(rng.normal(size=(9, 6)))
        response = basis * np.linspace(1.0, 3.0, 6)
        rest_center = rng.normal(size=9)

        rest = rest_center + rng.normal(scale=0.002, size=(300, 9))
        captures = {}
        for axis_index, axis in enumerate(cal.AXES):
            plus = rest_center + response[:, axis_index] + rng.normal(scale=0.002, size=(120, 9))
            minus = rest_center - response[:, axis_index] + rng.normal(scale=0.002, size=(120, 9))
            captures[f"+{axis}"] = plus
            captures[f"-{axis}"] = minus

        result = cal.compute_calibration(rest, captures, target_output=300.0, ridge=0.0)

        self.assertTrue(result.can_generate_header)
        self.assertEqual(result.rank, 6)
        np.testing.assert_allclose(result.confusion_matrix, np.eye(6), atol=0.02)

    def test_rank_deficient_calibration_is_rejected(self):
        rng = np.random.default_rng(5678)
        response = rng.normal(size=(9, 6))
        response[:, 5] = response[:, 4]
        rest_center = rng.normal(size=9)

        rest = rest_center + rng.normal(scale=0.001, size=(200, 9))
        captures = {}
        for axis_index, axis in enumerate(cal.AXES):
            plus = rest_center + response[:, axis_index] + rng.normal(scale=0.001, size=(80, 9))
            minus = rest_center - response[:, axis_index] + rng.normal(scale=0.001, size=(80, 9))
            captures[f"+{axis}"] = plus
            captures[f"-{axis}"] = minus

        result = cal.compute_calibration(rest, captures, target_output=300.0, ridge=0.03)

        self.assertFalse(result.can_generate_header)
        self.assertLess(result.rank, 6)

    def test_header_generation_format(self):
        result = cal.CalibrationResult(
            rest_mean=np.zeros(9),
            rest_std=np.zeros(9),
            response_matrix=np.eye(9, 6),
            decoder_matrix=np.ones((6, 9)),
            output_matrix=np.ones((6, 9)) * 1.25,
            deadzone=np.arange(6, dtype=float) + 5.0,
            rank=6,
            condition_number=1.0,
            singular_values=np.ones(6),
            confusion_matrix=np.eye(6),
            normalized_confusion_matrix=np.eye(6),
            signal_to_noise=np.ones(6) * 10.0,
            warnings=[],
            can_generate_header=True,
        )

        header = cal.generate_header(result, "2026-05-22T00:00:00+00:00")

        self.assertIn("constexpr bool AVAILABLE = true;", header)
        self.assertIn("constexpr float MATRIX[6][9]", header)
        self.assertIn("constexpr float DEADZONE[6]", header)
        self.assertIn("1.25f", header)


if __name__ == "__main__":
    unittest.main()
