import unittest

import numpy as np

from cam_calc import (
    CamParameters,
    FULL_TURN,
    calculate_flat_cam,
    calculate_signed_curvature,
    complete_cam_profile,
    create_parametric_spline,
    curvature_from_radius,
    flat_follower_radius_from_curvature,
    signed_curvature_radius,
)
from path_tasks import (
    DirectedRectanglePassageConstraint,
    ForbiddenPolygonConstraint,
)
from path_plan import PathPlannerThread, cam_smoothness_cost


class CamCoreTests(unittest.TestCase):
    def test_selectable_spline_degree_keeps_endpoints(self):
        points = np.array(
            [[0.0, 0.0], [1.0, 0.5], [2.0, -0.25], [3.0, 0.0],
             [4.0, 0.5], [5.0, 0.0]]
        )
        for requested_degree in (3, 4, 5):
            spline = create_parametric_spline(
                points, degree=requested_degree
            )
            self.assertEqual(spline.k, requested_degree)
            np.testing.assert_allclose(spline(0.0), points[0])
            np.testing.assert_allclose(spline(1.0), points[-1])
            np.testing.assert_array_equal(spline.t[:requested_degree + 1], 0.0)
            np.testing.assert_array_equal(spline.t[-requested_degree - 1:], 1.0)

    def test_two_waypoints_automatically_reduce_degree(self):
        points = np.array([[0.0, 0.0], [1.0, 0.2], [2.0, 0.2], [3.0, 0.0]])
        spline = create_parametric_spline(points, degree=5)
        self.assertEqual(spline.k, 3)

    def test_counterclockwise_circle_has_positive_curvature(self):
        radius = 4.0

        class Circle:
            def __call__(self, t, nu=0):
                if nu == 1:
                    return np.column_stack((-radius * np.sin(t), radius * np.cos(t)))
                if nu == 2:
                    return np.column_stack((-radius * np.cos(t), -radius * np.sin(t)))
                return np.column_stack((radius * np.cos(t), radius * np.sin(t)))

        curvature = calculate_signed_curvature(Circle(), np.linspace(0, 1, 20))
        np.testing.assert_allclose(curvature, 1.0 / radius)

    def test_zero_curvature_uses_positive_infinity(self):
        radius = signed_curvature_radius(np.array([-1e-12, 0.0, 1e-12]))
        np.testing.assert_array_equal(radius, np.array([np.inf, np.inf, np.inf]))

    def test_curvature_radius_is_converted_only_at_import_boundary(self):
        radius = np.array([-500.0, np.inf, 250.0])
        curvature = curvature_from_radius(radius)
        np.testing.assert_allclose(curvature, np.array([-0.002, 0.0, 0.004]))

    def test_curvature_radius_round_trip_keeps_small_curvature(self):
        curvature = np.array([-0.002, 0.0, 1e-6, 0.004])
        restored = curvature_from_radius(signed_curvature_radius(curvature))
        np.testing.assert_allclose(restored, curvature)

    def test_flat_formula_uses_signed_curvature(self):
        params = CamParameters(m=3.0, E=20.0, L=100.0, e=70.0)
        curvature = np.array([-0.002, 0.0, 0.002])
        result = flat_follower_radius_from_curvature(curvature, params)
        expected = params.e - params.E * params.L * curvature / (1 - params.m * curvature)
        np.testing.assert_allclose(result, expected)
        self.assertGreater(result[0], params.e)
        self.assertEqual(result[1], params.e)
        self.assertLess(result[2], params.e)

    def test_flat_formula_rejects_true_singularity(self):
        params = CamParameters(m=30.0)
        with self.assertRaises(ValueError):
            flat_follower_radius_from_curvature(
                np.array([1.0 / params.m]), params
            )

    def test_linear_closure_returns_to_start_radius(self):
        theta = np.array([0.0, 0.5, 1.0])
        radius = np.array([70.0, 68.0, 66.0])
        theta_full, radius_full, theta_closure, _ = complete_cam_profile(
            theta, radius, "linear"
        )
        self.assertAlmostEqual(theta_full[-1], FULL_TURN)
        self.assertAlmostEqual(radius_full[-1], radius[0])
        self.assertGreater(len(theta_closure), 1)

    def test_no_closure_keeps_only_work_profile(self):
        params = CamParameters(n=42.0, r0=65.0, closure_mode="none")
        x = np.linspace(0.0, 100.0, 101)
        result = calculate_flat_cam(x, np.zeros_like(x), np.zeros_like(x), params)
        np.testing.assert_array_equal(result["cam_angles"], result["work_angles"])
        self.assertEqual(len(result["closure_angles"]), 0)
        self.assertEqual(len(result["virtual_path"]), 0)

    def test_zero_curvature_closure_creates_straight_virtual_path(self):
        params = CamParameters(
            n=100.0, r0=10.0, closure_mode="linear"
        )
        x = np.linspace(0.0, 100.0, 101)
        result = calculate_flat_cam(x, np.zeros_like(x), np.zeros_like(x), params)
        virtual_path = result["virtual_path"]
        expected_length = params.n * params.r0 * (
            FULL_TURN - result["total_work_angle"]
        )
        self.assertGreater(len(virtual_path), 0)
        np.testing.assert_allclose(virtual_path[:, 1], 0.0, atol=1e-10)
        self.assertAlmostEqual(virtual_path[-1, 0], x[-1] + expected_length)

    def test_virtual_path_uses_curvature_recovered_from_cam_radius(self):
        curvature = np.full(101, 0.002)
        x = np.linspace(0.0, 100.0, len(curvature))
        result = calculate_flat_cam(
            x,
            np.zeros_like(x),
            curvature,
            CamParameters(closure_mode="hold"),
        )
        np.testing.assert_allclose(result["closure_curvature"], curvature[0])

    def test_straight_path_cam_uses_reference_radius(self):
        x = np.linspace(0.0, 100.0, 101)
        y = np.zeros_like(x)
        curvature = np.zeros_like(x)
        params = CamParameters(e=73.02, closure_mode="linear")
        result = calculate_flat_cam(x, y, curvature, params)
        np.testing.assert_allclose(result["work_radii"], params.e)
        self.assertAlmostEqual(result["cam_angles"][-1], FULL_TURN)

    def test_cam_work_angle_changes_with_path_length(self):
        params = CamParameters(n=42.0, r0=65.0)
        short_x = np.linspace(0.0, 100.0, 101)
        long_x = np.linspace(0.0, 200.0, 101)
        short = calculate_flat_cam(
            short_x, np.zeros_like(short_x), np.zeros_like(short_x), params
        )
        long = calculate_flat_cam(
            long_x, np.zeros_like(long_x), np.zeros_like(long_x), params
        )
        self.assertAlmostEqual(
            long["total_work_angle"], 2.0 * short["total_work_angle"]
        )
        self.assertAlmostEqual(
            short["work_angles"][-1], short["total_work_angle"]
        )
        self.assertAlmostEqual(
            long["work_angles"][-1], long["total_work_angle"]
        )

    def test_cam_smoothness_is_stable_at_different_sample_counts(self):
        costs = []
        for sample_count in (1001, 4001):
            angles = np.linspace(0.0, 2.0, sample_count)
            radii = 73.02 + 5.0 * np.sin(2.0 * angles)
            cost, _, _ = cam_smoothness_cost(angles, radii, 73.02)
            costs.append(cost)
        self.assertAlmostEqual(costs[0], costs[1], delta=1e-4)

    def test_waypoint_distance_uses_path_segments(self):
        points = np.array([[0.0, 0.0], [10.0, 0.0]])
        waypoints = np.array([[5.0, 3.0]])
        distances = PathPlannerThread._waypoint_distances(points, waypoints)
        np.testing.assert_allclose(distances, np.array([3.0]))

    def test_path_keeps_endpoints_and_initial_boundary(self):
        waypoints = np.array([[10.0, 20.0], [-300.0, 80.0], [-500.0, 100.0]])
        planner = PathPlannerThread(
            waypoints, start_angle_deg=180.0, straight_length=200.0
        )
        control_points = planner._create_control_points()
        self.assertEqual(len(control_points), 3 * (len(waypoints) - 1) + 1)
        np.testing.assert_allclose(control_points[0], waypoints[0])
        np.testing.assert_array_equal(control_points[-1], waypoints[-1])

        spline = create_parametric_spline(
            control_points, degree=planner.spline_degree
        )
        planner.spline_knots = spline.t.copy()
        planner.actual_spline_degree = spline.k
        fixed_knots = spline.t.copy()
        moved_points = control_points.copy()
        moved_points[3] += np.array([20.0, -10.0])
        moved_spline = create_parametric_spline(
            moved_points,
            degree=planner.actual_spline_degree,
            knots=planner.spline_knots,
        )
        np.testing.assert_array_equal(moved_spline.t, fixed_knots)
        tangent = spline(0.0, nu=1)
        tangent /= np.linalg.norm(tangent)
        np.testing.assert_allclose(tangent, np.array([-1.0, 0.0]), atol=1e-12)
        self.assertAlmostEqual(
            float(calculate_signed_curvature(spline, np.array([0.0]))[0]),
            0.0,
            places=12,
        )

        path, curvature = planner._sample_path(control_points, 2000)
        np.testing.assert_allclose(path[0], np.array([210.0, 20.0]))
        np.testing.assert_allclose(path[-1], waypoints[-1])
        join = int(np.argmin(np.linalg.norm(path - control_points[0], axis=1)))
        line_length = np.sum(np.linalg.norm(np.diff(path[:join + 1], axis=0), axis=1))
        self.assertAlmostEqual(line_length, 200.0, places=8)
        np.testing.assert_allclose(curvature[:join + 1], 0.0, atol=1e-12)
        cam_data = calculate_flat_cam(
            path[:join + 1, 0], path[:join + 1, 1],
            curvature[:join + 1], CamParameters()
        )
        np.testing.assert_allclose(
            cam_data["work_radii"], CamParameters().e, atol=1e-10
        )

    def test_forbidden_polygon_detects_inside_points(self):
        constraint = ForbiddenPolygonConstraint(
            np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]])
        )
        points = np.array([[-1.0, 1.0], [1.0, 1.0], [3.0, 1.0]])
        penalty, violation = constraint.evaluate(points, np.linspace(0, 1, 3))
        self.assertGreater(penalty, 0.0)
        self.assertGreater(violation, 0.0)

    def test_directed_rectangle_rejects_long_side_crossing(self):
        constraint = DirectedRectanglePassageConstraint(
            np.array([0.0, 0.0]), np.array([10.0, 0.0]), half_width=2.0
        )
        valid = np.column_stack((np.linspace(-1, 11, 100), np.zeros(100)))
        invalid = np.column_stack((np.full(100, 5.0), np.linspace(-3, 3, 100)))
        valid_penalty, _ = constraint.evaluate(valid, np.linspace(0, 1, 100))
        invalid_penalty, _ = constraint.evaluate(invalid, np.linspace(0, 1, 100))
        self.assertAlmostEqual(valid_penalty, 0.0)
        self.assertGreater(invalid_penalty, valid_penalty)


if __name__ == "__main__":
    unittest.main()
