"""Lamello Zeta P2 reference holes - Autodesk Fusion add-in.

Adds a "Zeta P2 Pins" command (Solid workspace > LAMELLO panel) that sketches
and optionally cuts the CNC reference holes used to position a Lamello Zeta P2
for Clamex P / P-System grooves:

* Joint mode   - select two or more touching panels; every touching pair is
                 analysed and gets end holes and/or face pin pairs.
* Single panel - select a face and a reference edge for parts whose mating
                 panel isn't modelled.

Sizes come from the design's user parameters (created if missing):
ZetaEnd_Pin, ZetaEnd_Inset, ZetaFace_Pin, ZetaFace_PinSpace, ZetaFace_Depth
and optionally ZetaEnd_Depth.
"""

import html
import json
import os
import traceback

import adsk.core
import adsk.fusion

from . import zeta_geometry as zg

CMD_ID = 'cgs_LamelloZetaPins'
CMD_NAME = 'Zeta P2 Pins'
CMD_TOOLTIP = ('Sketch and drill Lamello Zeta P2 CNC reference holes '
               '(Clamex P / P-System) for touching panels.')
WORKSPACE_ID = 'FusionSolidEnvironment'
TAB_ID = 'SolidTab'
PANEL_ID = 'cgs_LamelloPanel'
PANEL_NAME = 'LAMELLO'
FALLBACK_PANEL_ID = 'SolidCreatePanel'
ATTR_GROUP = 'cgs_LamelloZeta'
TIMELINE_GROUP_NAME = 'Lamello Zeta Pins'

PARAM_DEFAULTS = (
    ('ZetaEnd_Pin', '6 mm', 'Zeta P2 end connection - clip hole diameter'),
    ('ZetaEnd_Inset', '7.5 mm', 'Zeta P2 end connection - hole inset from edge'),
    ('ZetaFace_Pin', '5 mm', 'Zeta P2 face connection - pin hole diameter'),
    ('ZetaFace_PinSpace', '101 mm', 'Zeta P2 face connection - pin spacing'),
    ('ZetaFace_Depth', '6 mm', 'Zeta P2 face connection - pin hole depth'),
)
SETTINGS_FROM_PARAMS = (
    ('end_pin_d', 'ZetaEnd_Pin'),
    ('end_inset', 'ZetaEnd_Inset'),
    ('face_pin_d', 'ZetaFace_Pin'),
    ('face_pin_space', 'ZetaFace_PinSpace'),
    ('face_depth', 'ZetaFace_Depth'),
)

MODE_JOINT = 'Joint - select touching panels'
MODE_SINGLE = 'Single panel - face + edge'
KIND_END = 'End connection (clip hole)'
KIND_FACE = 'Face connection (pin pair)'
LAYOUT_SPACING = 'Max spacing'
LAYOUT_COUNT = 'Fixed count'
DEPTH_HALF = 'Half panel thickness'
DEPTH_PARAM = 'ZetaEnd_Depth parameter'

_app = None
_ui = None
_handlers = []        # add-in lifetime handlers
_cmd_handlers = []    # handlers for the currently open command
_inputs = {}

# Remembered between runs while Fusion is open (lengths in cm).
_last = {
    'mode': MODE_JOINT, 'kind': KIND_END, 'offset': 0.9, 'edgeDist': 7.5,
    'layout': LAYOUT_SPACING, 'count': 2, 'maxSpacing': 30.0, 'cut': True,
    'flip': False, 'depthMode': DEPTH_HALF,
}


# --------------------------------------------------------------------------
# Add-in entry points
# --------------------------------------------------------------------------

def run(context):
    global _app, _ui
    try:
        _app = adsk.core.Application.get()
        _ui = _app.userInterface

        cmd_def = _ui.commandDefinitions.itemById(CMD_ID)
        if not cmd_def:
            resources = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     'resources')
            cmd_def = _ui.commandDefinitions.addButtonDefinition(
                CMD_ID, CMD_NAME, CMD_TOOLTIP, resources)
        on_created = _CommandCreatedHandler()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)

        panel = _toolbar_panel(create=True)
        if panel and not panel.controls.itemById(CMD_ID):
            control = panel.controls.addCommand(cmd_def)
            control.isPromoted = True
            control.isPromotedByDefault = True
    except Exception:
        if _ui:
            _ui.messageBox('Zeta P2 Pins failed to start:\n{}'.format(
                traceback.format_exc()))


def stop(context):
    try:
        for panel in (_toolbar_panel(create=False),
                      _ui.allToolbarPanels.itemById(FALLBACK_PANEL_ID)):
            if not panel:
                continue
            control = panel.controls.itemById(CMD_ID)
            if control:
                control.deleteMe()
            if panel.id == PANEL_ID and panel.controls.count == 0:
                panel.deleteMe()
        cmd_def = _ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()
    except Exception:
        if _ui:
            _ui.messageBox('Zeta P2 Pins failed to stop:\n{}'.format(
                traceback.format_exc()))
    finally:
        _handlers.clear()
        _cmd_handlers.clear()
        _inputs.clear()


def _toolbar_panel(create):
    workspace = _ui.workspaces.itemById(WORKSPACE_ID)
    tab = workspace.toolbarTabs.itemById(TAB_ID) if workspace else None
    if tab:
        panel = tab.toolbarPanels.itemById(PANEL_ID)
        if panel:
            return panel
        if create:
            try:
                return tab.toolbarPanels.add(PANEL_ID, PANEL_NAME)
            except Exception:
                pass
    return _ui.allToolbarPanels.itemById(FALLBACK_PANEL_ID) if create else None


# --------------------------------------------------------------------------
# Fusion <-> geometry helpers
# --------------------------------------------------------------------------

def _design():
    return adsk.fusion.Design.cast(_app.activeProduct)


def _is_parametric(design):
    return design.designType == adsk.fusion.DesignTypes.ParametricDesignType


def _native(entity):
    return entity.nativeObject if entity.assemblyContext else entity


def _xyz(point):
    return (point.x, point.y, point.z)


def _point(xyz, matrix=None):
    p = adsk.core.Point3D.create(xyz[0], xyz[1], xyz[2])
    if matrix is not None:
        p.transformBy(matrix)
    return p


def _to_native_matrix(entity):
    """World -> component-space matrix for an entity inside an occurrence."""
    occ = entity.assemblyContext
    if not occ:
        return None
    matrix = occ.transform2.copy()
    matrix.invert()
    return matrix


def _check_transform(body, to_native):
    """Guard against drilling in the wrong place: a proxy vertex mapped back
    into component space must land on the matching native vertex."""
    if body.vertices.count == 0:
        return
    mapped = body.vertices.item(0).geometry.copy()
    mapped.transformBy(to_native)
    native = body.nativeObject.vertices.item(0).geometry
    if mapped.distanceTo(native) > 1e-4:
        raise RuntimeError('Could not map {} from assembly to component '
                           'coordinates.'.format(body.name))


def _panel_name(body):
    occ = body.assemblyContext
    if not occ:
        return body.name
    if occ.component.bRepBodies.count > 1:
        return '{}/{}'.format(occ.name, body.name)
    return occ.name


def _planar_face(face, fid):
    geom = face.geometry
    if geom.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
        return None
    normal = _xyz(geom.normal)
    if face.isParamReversed:
        normal = zg.v_neg(normal)
    verts = [_xyz(face.vertices.item(i).geometry)
             for i in range(face.vertices.count)]
    if not verts:
        return None
    edges = []
    for i in range(face.edges.count):
        edge = face.edges.item(i)
        if edge.geometry.curveType == adsk.core.Curve3DTypes.Line3DCurveType:
            edges.append((_xyz(edge.startVertex.geometry),
                          _xyz(edge.endVertex.geometry)))
    return zg.PlanarFace(fid, zg.v_norm(normal), _xyz(geom.origin), verts, edges)


class _Analysis:
    def __init__(self):
        self.plans = []
        self.notes = []
        self.error = None
        self.face_objs = {}      # face id -> BRepFace (as selected / proxy)
        self.to_native = {}      # panel key -> Matrix3D or None


def _linked_occurrence(body):
    """The first linked (external) occurrence in the body's path, if any."""
    occ = body.assemblyContext
    for _ in range(64):   # guard against an unexpected cycle
        if not occ:
            return None
        if occ.isReferencedComponent:
            return occ
        occ = occ.assemblyContext
    return None


def _add_panel(analysis, body, key):
    occ = body.assemblyContext
    linked = _linked_occurrence(body)
    if linked:
        raise zg.JointError('{} is inside a linked (external) component ({}) - '
                            'open it or break the link to add holes.'.format(
                                _panel_name(body), linked.name))
    to_native = _to_native_matrix(body)
    if to_native is not None:
        _check_transform(body, to_native)
    analysis.to_native[key] = to_native
    faces = []
    for i in range(body.faces.count):
        face = body.faces.item(i)
        pf = _planar_face(face, (key, i))
        if pf:
            faces.append(pf)
            analysis.face_objs[pf.id] = face
    verts = [_xyz(body.vertices.item(i).geometry)
             for i in range(body.vertices.count)]
    if occ:
        copies = _design().rootComponent.allOccurrencesByComponent(occ.component).count
        if copies > 1:
            analysis.notes.append(
                '{} is used {} times - holes added to it appear on every '
                'copy.'.format(occ.component.name, copies))
    return zg.Panel(key, _panel_name(body), verts, faces)


def _make_on_face(analysis):
    def on_face(face_id, xyz):
        face = analysis.face_objs.get(face_id)
        if face is None:
            return False
        evaluator = _native(face).evaluator
        ok, param = evaluator.getParameterAtPoint(
            _point(xyz, analysis.to_native.get(face_id[0])))
        return bool(ok and evaluator.isParameterOnFace(param))
    return on_face


def _param(design, name):
    try:
        return design.userParameters.itemByName(name)
    except Exception:
        return None


def _settings(design):
    s = zg.Settings()
    for attr, name in SETTINGS_FROM_PARAMS:
        p = _param(design, name)
        if p:
            setattr(s, attr, p.value)
    if _inputs['depthMode'].selectedItem.name == DEPTH_PARAM:
        p = _param(design, 'ZetaEnd_Depth')
        if p:
            s.end_depth = p.value
            s.end_depth_param = 'ZetaEnd_Depth'
    s.edge_distance = _inputs['edgeDist'].value
    s.layout_mode = ('count' if _inputs['layout'].selectedItem.name == LAYOUT_COUNT
                     else 'spacing')
    s.count = _inputs['count'].value
    s.max_spacing = _inputs['maxSpacing'].value
    s.flip = _inputs['flip'].value
    return s


def _analyze():
    analysis = _Analysis()
    design = _design()
    if not design:
        analysis.error = 'Open a design first.'
        return analysis
    s = _settings(design)
    if (_inputs['depthMode'].selectedItem.name == DEPTH_PARAM
            and not _param(design, 'ZetaEnd_Depth')):
        analysis.notes.append('No ZetaEnd_Depth parameter in this design - end '
                              'holes use half the panel thickness.')
    try:
        if _inputs['mode'].selectedItem.name == MODE_JOINT:
            sel = _inputs['panels']
            panels = []
            for i in range(sel.selectionCount):
                body = adsk.fusion.BRepBody.cast(sel.selection(i).entity)
                if body:
                    panels.append(_add_panel(analysis, body, i))
            if len(panels) < 2:
                analysis.error = 'Select two or more panels that touch.'
                return analysis
            plans, notes = zg.plan_panels(panels, s, _make_on_face(analysis))
            analysis.plans = plans
            analysis.notes.extend(notes)
            if not plans and not notes:
                analysis.error = 'None of the selected panels touch each other.'
        else:
            if _inputs['face'].selectionCount == 0 or _inputs['edge'].selectionCount == 0:
                analysis.error = 'Select the face to drill and a reference edge.'
                return analysis
            face = adsk.fusion.BRepFace.cast(_inputs['face'].selection(0).entity)
            edge = adsk.fusion.BRepEdge.cast(_inputs['edge'].selection(0).entity)
            if not face or not edge:
                analysis.error = 'Select a flat face and a straight edge.'
                return analysis
            panel = _add_panel(analysis, face.body, 0)
            pf = _planar_face(face, (0, 'selected'))
            if pf is None:
                analysis.error = 'The drill face must be flat.'
                return analysis
            analysis.face_objs[pf.id] = face
            kind = 'end' if _inputs['kind'].selectedItem.name == KIND_END else 'face'
            analysis.plans = [zg.plan_single(
                panel, pf,
                (_xyz(edge.startVertex.geometry), _xyz(edge.endVertex.geometry)),
                kind, s, _inputs['offset'].value, _make_on_face(analysis))]
    except (zg.JointError, RuntimeError) as e:
        analysis.error = str(e)
    return analysis


# --------------------------------------------------------------------------
# Building sketches and holes
# --------------------------------------------------------------------------

def _ensure_params(design):
    for name, expr, comment in PARAM_DEFAULTS:
        if not _param(design, name):
            design.userParameters.add(
                name, adsk.core.ValueInput.createByString(expr), 'mm', comment)


class _CutJob:
    def __init__(self, sketch, comp, body, direction, hs):
        self.sketch = sketch
        self.comp = comp
        self.body = body
        self.token = body.entityToken
        self.direction = direction
        self.hs = hs


def _build(analysis, preview):
    design = _design()
    parametric = _is_parametric(design)
    cut = _inputs['cut'].value and not preview
    start = None
    if not preview and parametric:
        _ensure_params(design)
        start = design.timeline.markerPosition
    # Pass 1: every sketch, while the stored face references are untouched.
    jobs = []
    for plan in analysis.plans:
        for hole_set in plan.hole_sets:
            job = _sketch_hole_set(analysis, hole_set, preview, parametric)
            if cut and job:
                jobs.append(job)
    # Pass 2: the cuts (they change the faces, so they go last).
    for job in jobs:
        _cut(design, job, parametric)
    if start is not None:
        try:
            end = design.timeline.markerPosition - 1
            if end > start:
                group = design.timeline.timelineGroups.add(start, end)
                group.name = TIMELINE_GROUP_NAME
        except Exception:
            pass   # grouping is cosmetic


def _sketch_hole_set(analysis, hs, preview, parametric):
    """Create the sketch; return a _CutJob if there is anything to cut."""
    face = _native(analysis.face_objs[hs.face_id])
    body = face.body
    comp = body.parentComponent
    to_native = analysis.to_native.get(hs.face_id[0])

    sketch = comp.sketches.addWithoutEdges(face)
    sketch.name = hs.label
    circles = []
    for xyz, on_face in zip(hs.centers, hs.on_face):
        sp = sketch.modelToSketchSpace(_point(xyz, to_native))
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(sp.x, sp.y, 0), hs.diameter / 2.0)
        if not on_face:
            circle.isConstruction = True
        circles.append(circle)
    for i, j in hs.pairs:
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(
            circles[i].centerSketchPoint, circles[j].centerSketchPoint)
        line.isConstruction = True
    if preview:
        return None

    try:
        _constrain(sketch, circles, hs, parametric)
    except Exception:
        pass   # constraints are a convenience; the geometry is already right
    try:
        sketch.attributes.add(ATTR_GROUP, 'holeSet', json.dumps({
            'kind': hs.kind, 'panel': hs.panel_name, 'count': len(circles)}))
    except Exception:
        pass
    if not any(hs.on_face):
        return None

    # Cut into the body, i.e. against the face's outward normal.
    # PositiveExtentDirection follows the sketch normal (x cross y, model space).
    sn = sketch.xDirection.crossProduct(sketch.yDirection)
    face_normal = _xyz(face.geometry.normal)
    if face.isParamReversed:
        face_normal = zg.v_neg(face_normal)
    direction = (adsk.fusion.ExtentDirections.NegativeExtentDirection
                 if zg.v_dot(_xyz(sn), face_normal) > 0
                 else adsk.fusion.ExtentDirections.PositiveExtentDirection)
    return _CutJob(sketch, comp, body, direction, hs)


def _constrain(sketch, circles, hs, parametric):
    if not circles:
        return
    for circle in circles:
        circle.centerSketchPoint.isFixed = True
    first = circles[0]
    for circle in circles[1:]:
        sketch.geometricConstraints.addEqual(first, circle)
    center = first.centerSketchPoint.geometry
    r = first.radius
    dim = sketch.sketchDimensions.addDiameterDimension(
        first, adsk.core.Point3D.create(center.x + 2 * r, center.y + 2 * r, 0))
    if parametric:
        dim.parameter.expression = hs.diameter_param


def _cut(design, job, parametric):
    hs = job.hs
    profiles = adsk.core.ObjectCollection.create()
    for i in range(job.sketch.profiles.count):
        profiles.add(job.sketch.profiles.item(i))
    if profiles.count == 0:
        return
    body = job.body
    if not body.isValid:   # an earlier cut may have replaced the reference
        found = design.findEntityByToken(job.token)
        body = found[0] if found else None
    extrudes = job.comp.features.extrudeFeatures
    ext_input = extrudes.createInput(
        profiles, adsk.fusion.FeatureOperations.CutFeatureOperation)
    if parametric and hs.depth_param:
        depth = adsk.core.ValueInput.createByString(hs.depth_param)
    else:
        depth = adsk.core.ValueInput.createByReal(hs.depth)
    ext_input.setOneSideExtent(
        adsk.fusion.DistanceExtentDefinition.create(depth), job.direction)
    if body:
        ext_input.participantBodies = [body]
    feature = extrudes.add(ext_input)
    feature.name = hs.label


# --------------------------------------------------------------------------
# Command dialog
# --------------------------------------------------------------------------

def _mm(cm):
    return '{:g} mm'.format(round(cm * 10.0, 2))


def _update_info(analysis):
    esc = html.escape
    lines = []
    if analysis.error:
        lines.append('<b>{}</b>'.format(esc(analysis.error)))
    for plan in analysis.plans:
        lines.append('<b>{}</b> - {} connector(s), joint {}'.format(
            esc(plan.label), plan.connector_count, _mm(plan.length)))
        for hs in plan.hole_sets:
            what = 'end hole(s)' if hs.kind == 'end' else 'pin hole(s)'
            lines.append('&nbsp;&nbsp;{} x &Oslash;{} {}, {} deep, on {}'.format(
                len(hs.centers), _mm(hs.diameter), what, _mm(hs.depth),
                esc(hs.panel_name)))
    for plan in analysis.plans:
        for warning in plan.warnings:
            lines.append('<font color="#c0392b">! {}</font>'.format(esc(warning)))
    for note in analysis.notes:
        lines.append('<i>{}</i>'.format(esc(note)))
    _inputs['info'].formattedText = '<br>'.join(lines) or 'Nothing selected yet.'


def _refresh_visibility():
    joint = _inputs['mode'].selectedItem.name == MODE_JOINT
    face_kind = _inputs['kind'].selectedItem.name == KIND_FACE
    count = _inputs['layout'].selectedItem.name == LAYOUT_COUNT
    _inputs['panels'].isVisible = joint
    _inputs['face'].isVisible = not joint
    _inputs['edge'].isVisible = not joint
    _inputs['kind'].isVisible = not joint
    _inputs['offset'].isVisible = (not joint) and face_kind
    _inputs['flip'].isVisible = joint
    _inputs['depthMode'].isVisible = joint or not face_kind
    _inputs['count'].isVisible = count
    _inputs['maxSpacing'].isVisible = not count


def _focus_selection():
    """Give focus to the visible selection input (hidden ones can't take it)."""
    joint = _inputs['mode'].selectedItem.name == MODE_JOINT
    _inputs['panels' if joint else 'face'].hasFocus = True


def _inputs_ready():
    if _inputs['mode'].selectedItem.name == MODE_JOINT:
        return _inputs['panels'].selectionCount >= 2
    return (_inputs['face'].selectionCount == 1 and
            _inputs['edge'].selectionCount == 1)


def _remember():
    _last['mode'] = _inputs['mode'].selectedItem.name
    _last['kind'] = _inputs['kind'].selectedItem.name
    _last['offset'] = _inputs['offset'].value
    _last['edgeDist'] = _inputs['edgeDist'].value
    _last['layout'] = _inputs['layout'].selectedItem.name
    _last['count'] = _inputs['count'].value
    _last['maxSpacing'] = _inputs['maxSpacing'].value
    _last['cut'] = _inputs['cut'].value
    _last['flip'] = _inputs['flip'].value
    _last['depthMode'] = _inputs['depthMode'].selectedItem.name


def _dropdown(inputs, input_id, name, items, selected):
    dd = inputs.addDropDownCommandInput(
        input_id, name, adsk.core.DropDownStyles.TextListDropDownStyle)
    for item in items:
        dd.listItems.add(item, item == selected, '')
    return dd


def _create_inputs(inputs, units):
    real = adsk.core.ValueInput.createByReal
    _inputs.clear()
    _inputs['mode'] = _dropdown(inputs, 'mode', 'Mode',
                                (MODE_JOINT, MODE_SINGLE), _last['mode'])

    panels = inputs.addSelectionInput(
        'panels', 'Panels', 'Select two or more panels that touch')
    panels.addSelectionFilter('SolidBodies')
    panels.setSelectionLimits(0, 0)
    _inputs['panels'] = panels

    face = inputs.addSelectionInput('face', 'Drill face',
                                    'Flat panel face the holes go into')
    face.addSelectionFilter('PlanarFaces')
    face.setSelectionLimits(0, 1)
    _inputs['face'] = face

    edge = inputs.addSelectionInput('edge', 'Reference edge',
                                    'Straight edge the holes are laid out along')
    edge.addSelectionFilter('LinearEdges')
    edge.setSelectionLimits(0, 1)
    _inputs['edge'] = edge

    _inputs['kind'] = _dropdown(inputs, 'kind', 'Connection',
                                (KIND_END, KIND_FACE), _last['kind'])
    offset = inputs.addValueInput('offset', 'Groove offset', units,
                                  real(_last['offset']))
    offset.tooltip = ('Distance from the reference edge to the groove '
                      'centreline, e.g. half the mating panel thickness.')
    _inputs['offset'] = offset

    layout = inputs.addGroupCommandInput('layoutGroup', 'Connector layout')
    layout.isExpanded = True
    li = layout.children
    edge_dist = li.addValueInput('edgeDist', 'Edge distance', units,
                                 real(_last['edgeDist']))
    edge_dist.tooltip = 'Distance from each end of the joint to the first/last connector.'
    _inputs['edgeDist'] = edge_dist
    _inputs['layout'] = _dropdown(li, 'layout', 'Spacing',
                                  (LAYOUT_SPACING, LAYOUT_COUNT), _last['layout'])
    _inputs['count'] = li.addIntegerSpinnerCommandInput(
        'count', 'Connectors', 1, 50, 1, _last['count'])
    _inputs['maxSpacing'] = li.addValueInput('maxSpacing', 'Max spacing', units,
                                             real(_last['maxSpacing']))

    options = inputs.addGroupCommandInput('optionsGroup', 'Options')
    options.isExpanded = True
    oi = options.children
    _inputs['cut'] = oi.addBoolValueInput('cut', 'Cut holes', True, '', _last['cut'])
    _inputs['flip'] = oi.addBoolValueInput('flip', 'Flip end-hole side', True, '',
                                           _last['flip'])
    _inputs['flip'].tooltip = ('End holes go on the inside face for corner joints '
                               'and the underside for T and edge joints. Tick to '
                               'use the opposite face.')
    _inputs['depthMode'] = _dropdown(oi, 'depthMode', 'End hole depth',
                                     (DEPTH_HALF, DEPTH_PARAM), _last['depthMode'])

    info = inputs.addTextBoxCommandInput('info', 'Summary', '', 8, True)
    info.isFullWidth = True
    _inputs['info'] = info


class _CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            design = _design()
            if not design:
                _ui.messageBox('Switch to the Design workspace with a design open.')
                return
            cmd = args.command
            cmd.isExecutedWhenPreEmpted = False
            _create_inputs(cmd.commandInputs, design.unitsManager.defaultLengthUnits)

            _cmd_handlers.clear()
            for event, handler in (
                    (cmd.execute, _ExecuteHandler()),
                    (cmd.executePreview, _PreviewHandler()),
                    (cmd.inputChanged, _InputChangedHandler()),
                    (cmd.validateInputs, _ValidateHandler())):
                event.add(handler)
                _cmd_handlers.append(handler)

            _refresh_visibility()
            _focus_selection()
            _update_info(_analyze())
        except Exception:
            _ui.messageBox('Zeta P2 Pins failed:\n{}'.format(traceback.format_exc()))


class _InputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            changed = args.input.id
            if (changed == 'face' and _inputs['face'].selectionCount == 1
                    and _inputs['edge'].selectionCount == 0):
                _inputs['edge'].hasFocus = True
            _refresh_visibility()
            if changed == 'mode':
                _focus_selection()
            _update_info(_analyze())
        except Exception:
            _inputs['info'].formattedText = html.escape(traceback.format_exc())


class _ValidateHandler(adsk.core.ValidateInputsEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            args.areInputsValid = _inputs_ready()
        except Exception:
            args.areInputsValid = False


class _PreviewHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            analysis = _analyze()
            if analysis.plans:
                _build(analysis, preview=True)
        except Exception:
            pass   # preview is best-effort; execute reports real errors


class _ExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            _remember()
            analysis = _analyze()
            if not analysis.plans:
                args.executeFailed = True
                args.executeFailedMessage = (analysis.error or '\n'.join(analysis.notes)
                                              or 'No joints found.')
                return
            _build(analysis, preview=False)
            warnings = [w for p in analysis.plans for w in p.warnings]
            if warnings:
                _ui.messageBox('Holes created, with warnings:\n\n- ' +
                               '\n- '.join(warnings), CMD_NAME)
        except Exception:
            args.executeFailed = True
            args.executeFailedMessage = traceback.format_exc()
