"""Pure-Python geometry for Lamello Zeta P2 CNC reference holes.

Nothing in this module imports the Fusion API, so it can be unit-tested
outside Fusion. All lengths are in centimetres (Fusion's internal unit)
and all points/vectors are plain (x, y, z) tuples in world space.

Rules implemented (from the shop's Zeta P2 parameters):

* END connection  - the panel whose end/edge face carries the P-System
  groove gets one hole per connector, drilled into one of its broad faces,
  `end_inset` in from the joint face, `end_pin_d` diameter, depth = half the
  panel thickness. The Zeta P2 positioning clip (Lamello 251067) registers in
  this hole, which is also the Clamex P access hole.
* FACE connection - the panel whose broad face receives the other panel's
  end gets a pair of holes per connector, `face_pin_space` apart, centred on
  the connector and on the mating panel's mid-thickness line. The Zeta P2
  positioning pins (Lamello 251048 / 251066) register in these holes.
"""

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

Vec = Tuple[float, float, float]

COPLANAR_TOL = 0.01       # 0.1 mm - faces closer than this count as touching
PARALLEL_TOL = 0.999      # |cos| above this counts as parallel
MIN_CONTACT_AREA = 0.25   # cm^2 - ignore line/corner touches
NARROW_RATIO = 0.6        # face short side < ratio * body depth => edge/end face


# --------------------------------------------------------------------------
# Vector helpers
# --------------------------------------------------------------------------

def v_add(a: Vec, b: Vec) -> Vec:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a: Vec, b: Vec) -> Vec:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_scale(a: Vec, s: float) -> Vec:
    return (a[0] * s, a[1] * s, a[2] * s)


def v_dot(a: Vec, b: Vec) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_cross(a: Vec, b: Vec) -> Vec:
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def v_len(a: Vec) -> float:
    return math.sqrt(v_dot(a, a))


def v_norm(a: Vec) -> Vec:
    length = v_len(a)
    if length < 1e-12:
        raise ValueError('zero-length vector')
    return v_scale(a, 1.0 / length)


def v_neg(a: Vec) -> Vec:
    return (-a[0], -a[1], -a[2])


def from_basis(u: Vec, t: Vec, n: Vec, su: float, st: float, sn: float) -> Vec:
    """World point from coordinates along an orthonormal basis (u, t, n)."""
    return v_add(v_add(v_scale(u, su), v_scale(t, st)), v_scale(n, sn))


def extent(points, axis: Vec) -> Tuple[float, float]:
    vals = [v_dot(p, axis) for p in points]
    return (min(vals), max(vals))


# --------------------------------------------------------------------------
# Data model
# --------------------------------------------------------------------------

@dataclass
class PlanarFace:
    id: object                 # opaque key the caller uses to find the real face
    normal: Vec                # outward unit normal
    origin: Vec                # any point on the plane
    vertices: List[Vec]
    linear_edges: List[Tuple[Vec, Vec]] = field(default_factory=list)


@dataclass
class Panel:
    key: object
    name: str
    vertices: List[Vec]
    faces: List[PlanarFace]


@dataclass
class Settings:
    end_pin_d: float = 0.6          # ZetaEnd_Pin      6 mm
    end_inset: float = 0.75         # ZetaEnd_Inset    7.5 mm
    face_pin_d: float = 0.5         # ZetaFace_Pin     5 mm
    face_pin_space: float = 10.1    # ZetaFace_PinSpace 101 mm
    face_depth: float = 0.6         # ZetaFace_Depth   6 mm
    end_depth: Optional[float] = None   # None => half the panel thickness
    end_depth_param: Optional[str] = None
    edge_distance: float = 7.5      # first/last connector centre from joint ends
    layout_mode: str = 'spacing'    # 'spacing' (max spacing) or 'count'
    count: int = 2
    max_spacing: float = 30.0
    flip: bool = False              # flip which broad face end holes go into


@dataclass
class HoleSet:
    kind: str                   # 'end' or 'face'
    panel_key: object
    panel_name: str
    face_id: object             # face the holes are sketched on / drilled into
    label: str
    centers: List[Vec]
    on_face: List[bool]
    diameter: float
    diameter_param: str
    depth: float
    depth_param: Optional[str]
    pairs: List[Tuple[int, int]] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


@dataclass
class JointPlan:
    label: str
    kind: str                   # 'end-to-face', 'end-to-end' or 'single'
    length: float
    connector_count: int
    hole_sets: List[HoleSet]
    warnings: List[str] = field(default_factory=list)


class JointError(Exception):
    """A pair of panels touches but can't be joined with this method."""


OnFace = Callable[[object, Vec], bool]


# --------------------------------------------------------------------------
# Face / panel analysis
# --------------------------------------------------------------------------

_WORLD_AXES = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def face_axes(face: PlanarFace) -> Tuple[Vec, Vec]:
    """In-plane axes (u, t): u along the face's longest straight edge."""
    n = face.normal
    best = None
    best_len = 0.0
    for p0, p1 in face.linear_edges:
        d = v_sub(p1, p0)
        d = v_sub(d, v_scale(n, v_dot(d, n)))       # project into the plane
        length = v_len(d)
        if length > best_len + 1e-9:
            best, best_len = d, length
    if best is None:
        # no straight edges - fall back to any in-plane direction
        axis = min(_WORLD_AXES, key=lambda a: abs(v_dot(a, n)))
        best = v_sub(axis, v_scale(n, v_dot(axis, n)))
    u = v_norm(best)
    t = v_norm(v_cross(n, u))
    return u, t


def classify_face(panel: Panel, face: PlanarFace) -> str:
    """'narrow' for an end/edge face, 'broad' for a show face."""
    u, t = face_axes(face)
    eu = extent(face.vertices, u)
    et = extent(face.vertices, t)
    short = min(eu[1] - eu[0], et[1] - et[0])
    en = extent(panel.vertices, face.normal)
    depth = en[1] - en[0]
    return 'narrow' if short < NARROW_RATIO * depth else 'broad'


def _bbox_on_face(face: PlanarFace, point: Vec, margin: float = 1e-4) -> bool:
    """Fallback point-in-face test: inside the face's in-plane bounding box."""
    u, t = face_axes(face)
    eu = extent(face.vertices, u)
    et = extent(face.vertices, t)
    pu, pt = v_dot(point, u), v_dot(point, t)
    return (eu[0] - margin <= pu <= eu[1] + margin and
            et[0] - margin <= pt <= et[1] + margin)


def make_default_on_face(panels: List[Panel]) -> OnFace:
    faces = {f.id: f for p in panels for f in p.faces}

    def on_face(face_id, point):
        return _bbox_on_face(faces[face_id], point)
    return on_face


@dataclass
class Contact:
    face_a: PlanarFace
    face_b: PlanarFace
    area: float
    class_a: str
    class_b: str


def find_contact(pa: Panel, pb: Panel) -> Optional[Contact]:
    """Largest supported face-to-face contact between two panels, or None."""
    candidates = []
    for fa in pa.faces:
        for fb in pb.faces:
            if v_dot(fa.normal, fb.normal) > -PARALLEL_TOL:
                continue
            if abs(v_dot(v_sub(fb.origin, fa.origin), fa.normal)) > COPLANAR_TOL:
                continue
            u, _ = face_axes(fa)
            w = v_cross(fa.normal, u)
            ua, ub = extent(fa.vertices, u), extent(fb.vertices, u)
            wa, wb = extent(fa.vertices, w), extent(fb.vertices, w)
            ou = min(ua[1], ub[1]) - max(ua[0], ub[0])
            ow = min(wa[1], wb[1]) - max(wa[0], wb[0])
            if ou <= COPLANAR_TOL or ow <= COPLANAR_TOL:
                continue
            area = ou * ow
            if area < MIN_CONTACT_AREA:
                continue
            candidates.append(Contact(fa, fb, area,
                                      classify_face(pa, fa),
                                      classify_face(pb, fb)))
    if not candidates:
        return None
    supported = [c for c in candidates
                 if not (c.class_a == 'broad' and c.class_b == 'broad')]
    pool = supported or candidates
    return max(pool, key=lambda c: c.area)


# --------------------------------------------------------------------------
# Layout
# --------------------------------------------------------------------------

def layout_positions(lo: float, hi: float, s: Settings) -> List[float]:
    """Connector centre positions along a joint running from lo to hi."""
    length = hi - lo
    if length <= 1e-6:
        return []
    mid = (lo + hi) / 2.0
    usable = length - 2.0 * s.edge_distance
    if usable <= 1e-6:
        return [mid]
    if s.layout_mode == 'count':
        n = max(1, int(s.count))
    else:
        spacing = max(s.max_spacing, 1e-6)
        n = max(2, int(math.ceil(usable / spacing - 1e-9)) + 1)
    if n == 1:
        return [mid]
    step = usable / (n - 1)
    return [lo + s.edge_distance + i * step for i in range(n)]


# --------------------------------------------------------------------------
# Hole planning
# --------------------------------------------------------------------------

def _down_side(t: Vec) -> int:
    """Pick the side whose normal points 'down' (-Z, else -Y, else -X)."""
    for axis in (_WORLD_AXES[2], _WORLD_AXES[1], _WORLD_AXES[0]):
        c = v_dot(t, axis)
        if abs(c) > 0.5:
            return -1 if c > 0 else 1
    return 1


def default_drill_side(end_face: PlanarFace, t: Vec, other: Panel) -> int:
    """Which broad face (+t or -t) of the end panel the end holes go into.

    Corner joints: the face toward the bulk of the mating panel (the inside
    of the carcass). T-joints and edge-to-edge joints: the downward face.
    """
    e0, e1 = extent(end_face.vertices, t)
    o0, o1 = extent(other.vertices, t)
    thick = e1 - e0
    more_plus = o1 - e1
    more_minus = e0 - o0
    near = 3.0 * thick
    if max(more_plus, more_minus) > near and min(more_plus, more_minus) < near:
        return 1 if more_plus > more_minus else -1
    return _down_side(t)


def _find_drill_face(panel: Panel, direction: Vec, point: Vec,
                     on_face: OnFace) -> Optional[PlanarFace]:
    best = None
    for f in panel.faces:
        if v_dot(f.normal, direction) < PARALLEL_TOL:
            continue
        if abs(v_dot(v_sub(point, f.origin), f.normal)) > COPLANAR_TOL:
            continue
        if on_face(f.id, point):
            return f
        best = best or f
    return best


def _end_holes(panel: Panel, contact_face: PlanarFace, u: Vec, t: Vec,
               side: int, positions: List[float], s: Settings,
               on_face: OnFace, partner: Panel) -> HoleSet:
    n = contact_face.normal                       # outward, toward the partner
    w = v_dot(contact_face.origin, n)
    t0, t1 = extent(contact_face.vertices, t)
    thick = t1 - t0
    t_face = t1 if side > 0 else t0
    centers = [from_basis(u, t, n, su, t_face, w - s.end_inset) for su in positions]
    drill_dir = v_scale(t, float(side))
    drill = _find_drill_face(panel, drill_dir, centers[0], on_face)
    if drill is None:
        raise JointError(
            '{}: no flat face to drill the end holes from '
            '(angled or mitred joints are not supported)'.format(panel.name))
    depth = s.end_depth if s.end_depth is not None else thick / 2.0
    hs = HoleSet(kind='end', panel_key=panel.key, panel_name=panel.name,
                 face_id=drill.id,
                 label='Zeta End - {} (to {})'.format(panel.name, partner.name),
                 centers=centers, on_face=[on_face(drill.id, c) for c in centers],
                 diameter=s.end_pin_d, diameter_param='ZetaEnd_Pin',
                 depth=depth,
                 depth_param=s.end_depth_param if s.end_depth is not None else None)
    if not all(hs.on_face):
        hs.warnings.append('{}: {} end hole(s) fall off the panel face'.format(
            panel.name, hs.on_face.count(False)))
    return hs


def _face_pins(panel: Panel, face: PlanarFace, u: Vec, t: Vec, n: Vec,
               w: float, t_mid: float, positions: List[float],
               cover: Optional[Tuple[float, float]], s: Settings,
               on_face: OnFace, partner: Panel) -> HoleSet:
    """Pin pairs on `face`. `cover` is the partner's extent along u; pins
    outside it end up visible after assembly (None skips that check)."""
    half = s.face_pin_space / 2.0
    r = s.face_pin_d / 2.0
    centers, pairs = [], []
    visible = 0
    for su in positions:
        a = from_basis(u, t, n, su - half, t_mid, w)
        b = from_basis(u, t, n, su + half, t_mid, w)
        pairs.append((len(centers), len(centers) + 1))
        centers += [a, b]
        if cover is not None:
            for pu in (su - half, su + half):
                if pu - r < cover[0] - 1e-6 or pu + r > cover[1] + 1e-6:
                    visible += 1
    hs = HoleSet(kind='face', panel_key=panel.key, panel_name=panel.name,
                 face_id=face.id,
                 label='Zeta Face - {} (from {})'.format(panel.name, partner.name),
                 centers=centers, on_face=[on_face(face.id, c) for c in centers],
                 diameter=s.face_pin_d, diameter_param='ZetaFace_Pin',
                 depth=s.face_depth, depth_param='ZetaFace_Depth', pairs=pairs)
    off = hs.on_face.count(False)
    if off:
        hs.warnings.append('{}: {} pin hole(s) fall off the panel - left as '
                           'construction circles, not cut'.format(panel.name, off))
    if visible:
        hs.warnings.append('{}: {} pin hole(s) extend past the end of {} and will '
                           'be visible - increase the edge distance'.format(
                               panel.name, visible, partner.name))
    return hs


def plan_joint(pa: Panel, pb: Panel, s: Settings,
               on_face: Optional[OnFace] = None) -> Optional[JointPlan]:
    """Plan the reference holes for one pair of panels.

    Returns None when the panels don't touch; raises JointError when they
    touch in a way this method can't handle.
    """
    on_face = on_face or make_default_on_face([pa, pb])
    c = find_contact(pa, pb)
    if c is None:
        return None
    if c.class_a == 'broad' and c.class_b == 'broad':
        raise JointError('{} and {} touch face-to-face (laminated), not at an '
                         'end - skipped'.format(pa.name, pb.name))
    if c.class_a == 'narrow':
        end_p, end_f, oth_p, oth_f, oth_class = pa, c.face_a, pb, c.face_b, c.class_b
    else:
        end_p, end_f, oth_p, oth_f, oth_class = pb, c.face_b, pa, c.face_a, c.class_a

    u, t = face_axes(end_f)
    n = end_f.normal
    w = v_dot(end_f.origin, n)
    eu_end = extent(end_f.vertices, u)
    eu_oth = extent(oth_f.vertices, u)
    lo, hi = max(eu_end[0], eu_oth[0]), min(eu_end[1], eu_oth[1])
    positions = layout_positions(lo, hi, s)
    if not positions:
        raise JointError('{} / {}: joint too short'.format(end_p.name, oth_p.name))

    side = default_drill_side(end_f, t, oth_p)
    if s.flip:
        side = -side

    hole_sets = [_end_holes(end_p, end_f, u, t, side, positions, s, on_face, oth_p)]
    if oth_class == 'broad':
        kind = 'end-to-face'
        t0, t1 = extent(end_f.vertices, t)
        hole_sets.append(_face_pins(oth_p, oth_f, u, t, n, w, (t0 + t1) / 2.0,
                                    positions, eu_end, s, on_face, end_p))
        label = '{} -> {}'.format(end_p.name, oth_p.name)
    else:
        kind = 'end-to-end'
        hole_sets.append(_end_holes(oth_p, oth_f, u, t, side, positions, s,
                                    on_face, end_p))
        label = '{} <-> {}'.format(end_p.name, oth_p.name)

    plan = JointPlan(label=label, kind=kind, length=hi - lo,
                     connector_count=len(positions), hole_sets=hole_sets)
    for hs in hole_sets:
        plan.warnings.extend(hs.warnings)
    return plan


def plan_panels(panels: List[Panel], s: Settings,
                on_face: Optional[OnFace] = None):
    """Plan every touching pair among the given panels.

    Returns (plans, notes): notes explain pairs that touch but were skipped.
    """
    on_face = on_face or make_default_on_face(panels)
    plans, notes = [], []
    for i in range(len(panels)):
        for j in range(i + 1, len(panels)):
            try:
                plan = plan_joint(panels[i], panels[j], s, on_face)
            except JointError as e:
                notes.append(str(e))
                continue
            if plan is not None:
                plans.append(plan)
    return plans, notes


def plan_single(panel: Panel, face: PlanarFace, edge: Tuple[Vec, Vec],
                kind: str, s: Settings, offset: float = 0.0,
                on_face: Optional[OnFace] = None) -> JointPlan:
    """Holes on one panel face, laid out along a reference edge.

    kind 'end':  one hole per connector, `end_inset` in from the edge.
    kind 'face': a pin pair per connector on a line `offset` in from the edge
                 (offset = distance from the edge to the groove centreline).
    """
    on_face = on_face or make_default_on_face([panel])
    n = face.normal
    p0, p1 = edge
    d = v_sub(p1, p0)
    d = v_sub(d, v_scale(n, v_dot(d, n)))
    if v_len(d) < 1e-6:
        raise JointError('The reference edge must run along the face, not into it')
    u = v_norm(d)
    inward = v_cross(n, u)
    centroid = v_scale(
        (sum(p[0] for p in face.vertices), sum(p[1] for p in face.vertices),
         sum(p[2] for p in face.vertices)), 1.0 / len(face.vertices))
    if v_dot(v_sub(centroid, p0), inward) < 0:
        inward = v_neg(inward)
    w = v_dot(face.origin, n)
    line = v_dot(p0, inward)

    if kind == 'end':
        lo, hi = sorted((v_dot(p0, u), v_dot(p1, u)))
        positions = layout_positions(lo, hi, s)
        if not positions:
            raise JointError('Reference edge is too short')
        centers = [from_basis(u, inward, n, su, line + s.end_inset, w)
                   for su in positions]
        en = extent(panel.vertices, n)
        thick = en[1] - en[0]
        depth = s.end_depth if s.end_depth is not None else thick / 2.0
        hs = HoleSet(kind='end', panel_key=panel.key, panel_name=panel.name,
                     face_id=face.id, label='Zeta End - {}'.format(panel.name),
                     centers=centers, on_face=[on_face(face.id, c) for c in centers],
                     diameter=s.end_pin_d, diameter_param='ZetaEnd_Pin',
                     depth=depth,
                     depth_param=s.end_depth_param if s.end_depth is not None else None)
        if not all(hs.on_face):
            hs.warnings.append('{}: {} end hole(s) fall off the face'.format(
                panel.name, hs.on_face.count(False)))
    else:
        lo, hi = extent(face.vertices, u)
        positions = layout_positions(lo, hi, s)
        if not positions:
            raise JointError('Face is too short along the reference edge')
        hs = _face_pins(panel, face, u, inward, n, w, line + offset, positions,
                        None, s, on_face, panel)
        hs.label = 'Zeta Face - {}'.format(panel.name)

    return JointPlan(label=hs.label, kind='single', length=hi - lo,
                     connector_count=len(positions), hole_sets=[hs],
                     warnings=list(hs.warnings))
