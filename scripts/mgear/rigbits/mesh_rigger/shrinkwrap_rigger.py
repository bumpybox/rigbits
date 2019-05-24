from functools import partial
import json

import pymel.core as pm

from mgear.core import skin, curve, icon
from . import lib
from mgear.vendor.Qt import QtWidgets
import mgear.core.pyqt as gqt
from mgear.rigbits import facial_rigger


def rig(*args, **kwargs):
    kwargs, organization_keys = lib.extract_organization_keys(kwargs)
    results = _rig(*args, **kwargs)
    lib.organize_results(results, **organization_keys)

    # Dont renaming master control
    nodes = []
    for node in results["controls_set"]:
        if not node.name().endswith("_master_ctrl"):
            nodes.append(node)

    prefix = ""
    if "prefix" in kwargs:
        prefix = kwargs["prefix"] + "_"
    lib.rename_by_position(
        nodes, prefix=prefix, suffix="_ctrl"
    )


def _rig(mesh=None,
         shrinkwrap_mesh=None,
         main_control_start=0,
         main_control_frequency=1,
         prefix="shrinkwrap_rig",
         hook_up_parent=None,
         control_size=1.0,
         control_offset=0.0):

    results = {"setup_group": [], "controls_group": [], "controls_set": []}

    mesh = pm.PyNode(mesh)

    boundary_verts = []
    connecting_edges = []
    for edge in mesh.edges:
        if not edge.isOnBoundary():
            connecting_edges.append(edge)
            boundary_verts.append(edge.connectedVertices()[1])

    ordered_verts = [boundary_verts[0]]
    for count in range(0, len(boundary_verts)):
        ordered_verts = lib.connected_verts(boundary_verts, ordered_verts)

    ordered_edges = []
    for vert in ordered_verts:
        for edge in vert.connectedEdges():
            if edge in connecting_edges:
                ordered_edges.append(edge)

    joints = []
    for edge in ordered_edges:
        # Place joints.
        up_vector_position = list(
            set(edge.connectedVertices()) & set(ordered_verts)
        )[0].getPosition(space="world")
        joint = lib.create_edge_joint(edge, up_vector_position)
        pm.rename(
            joint,
            "{0}_shrinkwrap{1:0>2}_jnt".format(
                prefix, ordered_edges.index(edge)
            )
        )
        joints.append(joint)

        results["setup_group"].append(joint)

    # Skin mesh. One connected_edge per joint.
    skinCluster = skin.getSkinCluster(mesh)
    if not skinCluster:
        skinCluster = pm.skinCluster(joints, mesh, toSelectedBones=True, nw=2)

    pm.skinPercent(skinCluster, mesh, pruneWeights=100, normalize=False)
    for edge in ordered_edges:
        joint = joints[ordered_edges.index(edge)]
        for vert in edge.connectedVertices():
            pm.skinPercent(skinCluster, vert, transformValue=[(joint, 1.0)])

    # Master control
    clusTfm = pm.cluster(ordered_verts)[1]
    center_position = clusTfm.rotatePivot.get()
    pm.delete(clusTfm)

    master_group = pm.group(name="{0}_master_grp".format(prefix), empty=True)
    master_group.setTranslation(center_position)
    if hook_up_parent:
        pm.parentConstraint(master_group, hook_up_parent)
    results["controls_group"].append(master_group)

    master_null = pm.duplicate(master_group)[0]
    pm.rename(master_null, "{0}_master_null".format(prefix))
    pm.parent(master_null, master_group)

    points = [x.getTranslation(space="world") for x in joints]
    master_control = curve.addCurve(
        master_null,
        "{0}_master_ctrl".format(prefix),
        points,
        close=True,
        degree=1
    )
    curve.set_color(master_control, [1, 1, 0])
    pm.move(
        master_control,
        [0, 0, control_offset],
        relative=True,
        objectSpace=True
    )
    pm.makeIdentity(master_control, apply=True)
    master_control.resetFromRestPosition()
    results["controls_set"].append(master_control)

    # Create controls with parent and children. Relationship is determined by
    # skipping edges in the ring. Starting point is configurable.
    if main_control_start / main_control_frequency == 1:
        main_control_start = 0

    # Parent controls
    parent_controls = []
    for joint in joints[main_control_start::main_control_frequency]:
        group = pm.group(
            name="{0}_main{1:0>2}_grp".format(prefix, joints.index(joint)),
            empty=True
        )
        group.setMatrix(joint.getMatrix())

        null = pm.group(
            name="{0}_main{1:0>2}_null".format(prefix, joints.index(joint)),
            empty=True
        )
        null.setMatrix(joint.getMatrix())

        control = icon.create(
            name="{0}_main{1:0>2}_ctrl".format(prefix, joints.index(joint)),
            icon="cube",
            color=[1, 0, 0]
        )
        control.setMatrix(group.getMatrix())
        pm.rotate(control, [0, 0, 90], relative=True, objectSpace=True)
        results["controls_set"].append(control)

        pm.parent(group, master_control)
        pm.parent(null, group)
        pm.parent(control, null)

        pm.move(
            control,
            [0, 0, control_offset],
            relative=True,
            objectSpace=True
        )
        pm.scale(control, [control_size, control_size, control_size])
        pm.makeIdentity(control, apply=True)
        control.resetFromRestPosition()
        pm.parentConstraint(control, joint)
        parent_controls.append(control)

    # Child controls
    parent_index = 0
    # Duplicate the parent controls to loop back around.
    parents = parent_controls + parent_controls
    for joint in joints:
        if joint in joints[main_control_start::main_control_frequency]:
            parent_index += 1
            continue

        group = pm.group(
            name="{0}_main{1:0>2}_grp".format(prefix, joints.index(joint)),
            empty=True
        )
        group.setMatrix(joint.getMatrix())

        null = pm.group(
            name="{0}_main{1:0>2}_null".format(prefix, joints.index(joint)),
            empty=True
        )
        null.setMatrix(joint.getMatrix())

        control = icon.create(
            name="{0}_main{1:0>2}_ctrl".format(prefix, joints.index(joint)),
            icon="sphere",
            color=[0, 1, 0]
        )
        control.setMatrix(group.getMatrix())
        pm.rotate(control, [0, 0, 90], relative=True, objectSpace=True)
        results["controls_set"].append(control)

        pm.parent(group, master_control)
        pm.parent(null, group)
        pm.parent(control, null)

        pm.move(
            control,
            [0, 0, control_offset],
            relative=True,
            objectSpace=True
        )
        pm.scale(control, [control_size, control_size, control_size])
        pm.makeIdentity(control, apply=True)
        control.resetFromRestPosition()

        pm.parentConstraint(control, joint)
        weight = parent_index - (
            float(joints.index(joint)) / main_control_frequency
        )
        pm.parentConstraint(
            parents[parent_index],
            group,
            weight=1.0 - weight,
            maintainOffset=True
        )
        pm.parentConstraint(
            parents[parent_index - 1],
            group,
            weight=weight,
            maintainOffset=True
        )

    # Setup shrinkwrap
    shrinkWrapNode = pm.deformer(mesh, type="shrinkWrap")[0]
    pm.PyNode(shrinkwrap_mesh).worldMesh[0] >> shrinkWrapNode.targetGeom
    shrinkWrapNode.projection.set(4)

    master_control.addAttr(
        "wobble_smooth",
        usedAsProxy=True,
        keyable=True,
        min=0,
        max=10
    )
    shrinkWrapNode.targetSmoothLevel.connect(master_control.wobble_smooth)
    master_control.wobble_smooth.set(2)

    return results


class ui(lib.settings_dialog):

    def __init__(self, parent=None):
        super(ui, self).__init__(parent)

        self.filter = (
            "Shrinkwrap Rigger Configuration .shrinkwrap (*.shrinkwrap)"
        )

        self.setWindowTitle("Shrinkwrap Rigger")

    def create_body_layout(self):

        # mesh
        self.mesh_label = QtWidgets.QLabel("Mesh:")
        self.mesh = QtWidgets.QLineEdit()
        self.mesh_button = QtWidgets.QPushButton("<<")
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.mesh_label)
        layout.addWidget(self.mesh)
        layout.addWidget(self.mesh_button)
        self.main_layout.addLayout(layout)

        self.mesh_button.clicked.connect(
            partial(self.populate_object, self.mesh)
        )

        # shrinkwrap_mesh
        self.shrinkwrap_mesh_label = QtWidgets.QLabel("Shrinkwrap Mesh:")
        self.shrinkwrap_mesh = QtWidgets.QLineEdit()
        self.shrinkwrap_mesh_button = QtWidgets.QPushButton("<<")
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.shrinkwrap_mesh_label)
        layout.addWidget(self.shrinkwrap_mesh)
        layout.addWidget(self.shrinkwrap_mesh_button)
        self.main_layout.addLayout(layout)

        self.shrinkwrap_mesh_button.clicked.connect(
            partial(self.populate_object, self.shrinkwrap_mesh)
        )

        # main_control_start
        self.main_control_start_label = QtWidgets.QLabel("Main Control Start:")
        self.main_control_start = QtWidgets.QSpinBox()
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.main_control_start_label)
        layout.addWidget(self.main_control_start)
        self.main_layout.addLayout(layout)

        # main_control_frequency
        self.main_control_frequency_label = QtWidgets.QLabel(
            "Main Control Frequency:"
        )
        self.main_control_frequency = QtWidgets.QSpinBox()
        self.main_control_frequency.setMinimum(1)
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.main_control_frequency_label)
        layout.addWidget(self.main_control_frequency)
        self.main_layout.addLayout(layout)

    def build_rig(self):
        rig(**facial_rigger.lib.get_settings_from_widget(self))


# Build from json file.
def rig_from_file(path):
    rig(**json.load(open(path)))


def show(*args):
    gqt.showDialog(ui)
