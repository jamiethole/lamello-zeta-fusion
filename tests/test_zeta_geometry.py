"""Unit tests for the Fusion-independent geometry engine.

Run from the project folder:  python3 -m unittest discover -s tests -v
All dimensions are in cm (Fusion's internal unit): 1.8 = 18 mm panel.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'LamelloZetaPins'))
import zeta_geometry as zg  # noqa: E402

T = 1.8   # 18 mm panels


def box(key, name, x0, x1, y0, y1, z0, z1):
    xs, ys, zs = (x0, x1), (y0, y1), (z0, z1)
    verts = [(x, y, z) for x in xs for y in ys for z in zs]
    faces = []
    for axis in range(3):
        for side, val in ((-1, (xs, ys, zs)[axis][0]), (1, (xs, ys, zs)[axis][1])):
            n = [0.0, 0.0, 0.0]
            n[axis] = float(side)
            fv = [v for v in verts if v[axis] == val]
            edges = [(a, b) for i, a in enumerate(fv) for b in fv[i + 1:]
                     if sum(1 for k in range(3) if a[k] != b[k]) == 1]
            faces.append(zg.PlanarFace((key, axis, side), tuple(n), fv[0], fv, edges))
    return zg.Panel(key, name, verts, faces)


def rotation(axis, deg):
    x, y, z = zg.v_norm(axis)
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    C = 1 - c
    return ((c + x * x * C, x * y * C - z * s, x * z * C + y * s),
            (y * x * C + z * s, c + y * y * C, y * z * C - x * s),
            (z * x * C - y * s, z * y * C + x * s, c + z * z * C))


def rot(m, p):
    return tuple(sum(m[r][k] * p[k] for k in range(3)) for r in range(3))


def rotate_panel(panel, m):
    faces = [zg.PlanarFace(f.id, rot(m, f.normal), rot(m, f.origin),
                           [rot(m, v) for v in f.vertices],
                           [(rot(m, a), rot(m, b)) for a, b in f.linear_edges])
             for f in panel.faces]
    return zg.Panel(panel.key, panel.name, [rot(m, v) for v in panel.vertices], faces)


def pts(points, nd=4):
    return sorted(tuple(round(c, nd) + 0.0 for c in p) for p in points)


def by_kind(plan, kind):
    return [hs for hs in plan.hole_sets if hs.kind == kind]


class LayoutTests(unittest.TestCase):
    def test_max_spacing(self):
        s = zg.Settings(edge_distance=7.5, max_spacing=30.0)
        self.assertEqual(zg.layout_positions(0, 40, s), [7.5, 32.5])
        got = zg.layout_positions(0, 80, s)          # 65 usable -> 4 connectors
        self.assertEqual(len(got), 4)
        self.assertAlmostEqual(got[0], 7.5)
        self.assertAlmostEqual(got[-1], 72.5)

    def test_exact_multiple_does_not_add_extra(self):
        s = zg.Settings(edge_distance=5.0, max_spacing=25.0)
        self.assertEqual(zg.layout_positions(0, 60, s), [5.0, 30.0, 55.0])

    def test_fixed_count(self):
        s = zg.Settings(edge_distance=5.0, layout_mode='count', count=3)
        self.assertEqual(zg.layout_positions(0, 60, s), [5.0, 30.0, 55.0])
        s.count = 1
        self.assertEqual(zg.layout_positions(0, 60, s), [30.0])

    def test_short_joint_gets_one_centred_connector(self):
        s = zg.Settings(edge_distance=7.5)
        self.assertEqual(zg.layout_positions(0, 10, s), [5.0])
        self.assertEqual(zg.layout_positions(5, 5, s), [])


class ClassifyTests(unittest.TestCase):
    def test_end_and_show_faces(self):
        side = box('side', 'Side', 0, T, 0, 40, 0, 70)
        faces = {f.id: f for f in side.faces}
        self.assertEqual(zg.classify_face(side, faces[('side', 2, 1)]), 'narrow')  # top end
        self.assertEqual(zg.classify_face(side, faces[('side', 1, 1)]), 'narrow')  # front edge
        self.assertEqual(zg.classify_face(side, faces[('side', 0, 1)]), 'broad')   # inside face


class JointTests(unittest.TestCase):
    def corner(self, **kw):
        top = box('top', 'Top', 0, 60, 0, 40, 70, 70 + T)
        side = box('side', 'Side', 0, T, 0, 40, 0, 70)
        return top, side, zg.plan_joint(top, side, zg.Settings(**kw))

    def test_corner_side_under_top(self):
        _, _, plan = self.corner()
        self.assertEqual(plan.kind, 'end-to-face')
        self.assertEqual(plan.connector_count, 2)
        self.assertAlmostEqual(plan.length, 40)
        self.assertEqual(plan.warnings, [])

        end, = by_kind(plan, 'end')
        self.assertEqual(end.panel_key, 'side')
        self.assertEqual(end.face_id, ('side', 0, 1))           # inside face (x = 1.8)
        self.assertEqual(pts(end.centers), pts([(T, 7.5, 69.25), (T, 32.5, 69.25)]))
        self.assertAlmostEqual(end.depth, 0.9)                 # half of 18 mm
        self.assertAlmostEqual(end.diameter, 0.6)
        self.assertTrue(all(end.on_face))

        face, = by_kind(plan, 'face')
        self.assertEqual(face.panel_key, 'top')
        self.assertEqual(face.face_id, ('top', 2, -1))          # underside of the top
        self.assertEqual(pts(face.centers), pts(
            [(0.9, y, 70) for y in (2.45, 12.55, 27.45, 37.55)]))
        self.assertAlmostEqual(face.depth, 0.6)
        self.assertAlmostEqual(face.diameter, 0.5)
        self.assertEqual(len(face.pairs), 2)
        for i, j in face.pairs:                                 # pairs are 101 mm apart
            self.assertAlmostEqual(zg.v_len(zg.v_sub(face.centers[i], face.centers[j])), 10.1)

    def test_order_of_selection_does_not_matter(self):
        top, side, plan = self.corner()
        again = zg.plan_joint(side, top, zg.Settings())
        self.assertEqual(pts(by_kind(plan, 'end')[0].centers),
                         pts(by_kind(again, 'end')[0].centers))
        self.assertEqual(pts(by_kind(plan, 'face')[0].centers),
                         pts(by_kind(again, 'face')[0].centers))

    def test_flip_moves_end_holes_to_outside_face(self):
        _, _, plan = self.corner(flip=True)
        end, = by_kind(plan, 'end')
        self.assertEqual(end.face_id, ('side', 0, -1))
        self.assertEqual(pts(end.centers), pts([(0, 7.5, 69.25), (0, 32.5, 69.25)]))

    def test_end_depth_parameter_overrides_half_thickness(self):
        _, _, plan = self.corner(end_depth=1.2, end_depth_param='ZetaEnd_Depth')
        end, = by_kind(plan, 'end')
        self.assertAlmostEqual(end.depth, 1.2)
        self.assertEqual(end.depth_param, 'ZetaEnd_Depth')

    def test_shelf_t_joint_drills_underside(self):
        side = box('side', 'Side', 0, T, 0, 40, 0, 70)
        shelf = box('shelf', 'Shelf', T, 60, 0, 38, 30, 30 + T)
        plan = zg.plan_joint(side, shelf, zg.Settings())
        end, = by_kind(plan, 'end')
        self.assertEqual(end.panel_key, 'shelf')
        self.assertEqual(end.face_id, ('shelf', 2, -1))
        self.assertEqual(pts(end.centers), pts([(T + 0.75, 7.5, 30), (T + 0.75, 30.5, 30)]))
        face, = by_kind(plan, 'face')
        self.assertEqual(face.face_id, ('side', 0, 1))
        self.assertEqual(pts(face.centers), pts(
            [(T, y, 30.9) for y in (2.45, 12.55, 25.45, 35.55)]))
        self.assertEqual(plan.warnings, [])

    def test_top_between_sides_drills_underside_of_top(self):
        left = box('left', 'Left', 0, T, 0, 40, 0, 70 + T)
        right = box('right', 'Right', 60 - T, 60, 0, 40, 0, 70 + T)
        top = box('top', 'Top', T, 60 - T, 0, 40, 70, 70 + T)
        plans, notes = zg.plan_panels([left, right, top], zg.Settings())
        self.assertEqual(len(plans), 2)
        self.assertEqual(notes, [])
        for plan in plans:
            end, = by_kind(plan, 'end')
            self.assertEqual(end.panel_key, 'top')
            self.assertEqual(end.face_id, ('top', 2, -1))
            for c in end.centers:
                self.assertAlmostEqual(c[2], 70)
            xs = {round(c[0], 4) for c in end.centers}
            self.assertTrue(xs == {round(T + 0.75, 4)} or xs == {round(60 - T - 0.75, 4)})

    def test_edge_to_edge_glue_up(self):
        a = box('a', 'A', 0, 30, 0, 100, 0, T)
        b = box('b', 'B', 30, 60, 0, 100, 0, T)
        plan = zg.plan_joint(a, b, zg.Settings())
        self.assertEqual(plan.kind, 'end-to-end')
        ends = by_kind(plan, 'end')
        self.assertEqual(len(ends), 2)
        xs = sorted(round(hs.centers[0][0], 4) for hs in ends)
        self.assertEqual(xs, [29.25, 30.75])
        for hs in ends:
            self.assertEqual(hs.face_id[1:], (2, -1))            # both on the underside
            self.assertEqual(len(hs.centers), plan.connector_count)

    def test_laminated_and_apart_panels_are_skipped(self):
        a = box('a', 'A', 0, 60, 0, 40, 0, T)
        b = box('b', 'B', 0, 60, 0, 40, T, 2 * T)
        far = box('far', 'Far', 100, 110, 0, 40, 0, T)
        self.assertIsNone(zg.plan_joint(a, far, zg.Settings()))
        plans, notes = zg.plan_panels([a, b, far], zg.Settings())
        self.assertEqual(plans, [])
        self.assertEqual(len(notes), 1)
        self.assertIn('laminated', notes[0])

    def test_pins_past_short_partner_are_flagged_visible(self):
        top = box('top', 'Top', 0, 60, -20, 40, 70, 70 + T)
        side = box('side', 'Side', 0, T, 0, 12, 0, 70)
        plan = zg.plan_joint(top, side, zg.Settings(edge_distance=2.0))
        self.assertTrue(any('visible' in w for w in plan.warnings))

    def test_pins_off_the_receiving_panel_are_flagged(self):
        top = box('top', 'Top', 0, 60, 0, 40, 70, 70 + T)
        side = box('side', 'Side', 0, T, 0, 40, 0, 70)
        plan = zg.plan_joint(top, side, zg.Settings(edge_distance=2.0))
        face, = by_kind(plan, 'face')
        self.assertIn(False, face.on_face)
        self.assertTrue(any('fall off' in w for w in plan.warnings))

    def test_rotated_assembly_gives_rotated_holes(self):
        m = rotation((1, 2, 3), 37)
        top = rotate_panel(box('top', 'Top', 0, 60, 0, 40, 70, 70 + T), m)
        side = rotate_panel(box('side', 'Side', 0, T, 0, 40, 0, 70), m)
        plan = zg.plan_joint(top, side, zg.Settings())
        end, = by_kind(plan, 'end')
        face, = by_kind(plan, 'face')
        self.assertEqual(end.face_id, ('side', 0, 1))
        self.assertEqual(pts(end.centers, 3), pts(
            [rot(m, p) for p in ((T, 7.5, 69.25), (T, 32.5, 69.25))], 3))
        self.assertEqual(pts(face.centers, 3), pts(
            [rot(m, (0.9, y, 70)) for y in (2.45, 12.55, 27.45, 37.55)], 3))


class SingleTests(unittest.TestCase):
    def setUp(self):
        self.panel = box('p', 'Panel', 0, 40, 0, 60, 0, T)
        self.face = next(f for f in self.panel.faces if f.id == ('p', 2, 1))
        self.edge = ((0, 0, T), (0, 60, T))

    def test_end_holes_along_edge(self):
        plan = zg.plan_single(self.panel, self.face, self.edge, 'end', zg.Settings())
        hs, = plan.hole_sets
        self.assertEqual(pts(hs.centers), pts([(0.75, y, T) for y in (7.5, 30, 52.5)]))
        self.assertAlmostEqual(hs.depth, 0.9)

    def test_face_pins_at_offset(self):
        plan = zg.plan_single(self.panel, self.face, self.edge, 'face',
                              zg.Settings(), offset=30.9)
        hs, = plan.hole_sets
        self.assertEqual(len(hs.centers), 6)
        self.assertTrue(all(abs(c[0] - 30.9) < 1e-9 for c in hs.centers))
        self.assertEqual(plan.warnings, [])

    def test_edge_direction_reversed_still_goes_inward(self):
        edge = ((40, 60, T), (40, 0, T))                       # opposite edge, reversed
        plan = zg.plan_single(self.panel, self.face, edge, 'end', zg.Settings())
        self.assertTrue(all(abs(c[0] - 39.25) < 1e-9 for c in plan.hole_sets[0].centers))


if __name__ == '__main__':
    unittest.main()
