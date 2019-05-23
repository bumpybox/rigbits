from functools import partial
import json

import pymel.core as pm
from maya.app.general.mayaMixin import MayaQWidgetDockableMixin

from mgear.core import skin, curve, icon
from . import lib
from mgear.rigbits import facial_rigger
from mgear.vendor.Qt import QtCore, QtWidgets
import mgear.core.pyqt as gqt


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
         size=1.0,
         offset=0.0):

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

    master_parent = pm.group(name="{0}_master_grp".format(prefix), empty=True)
    master_parent.setTranslation(center_position)
    if hook_up_parent:
        pm.parentConstraint(master_parent, hook_up_parent)
    results["controls_group"].append(master_parent)

    points = [x.getTranslation(space="world") for x in joints]
    master_control = curve.addCurve(
        master_parent,
        "{0}_master_ctrl".format(prefix),
        points,
        close=True,
        degree=1
    )
    curve.set_color(master_control, [1, 1, 0])
    pm.move(master_control, [0, 0, offset], relative=True, objectSpace=True)
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

        control = icon.create(
            name="{0}_main{1:0>2}_ctrl".format(prefix, joints.index(joint)),
            icon="cube",
            color=[1, 0, 0]
        )
        control.setMatrix(group.getMatrix())
        pm.rotate(control, [0, 0, 90], relative=True, objectSpace=True)
        results["controls_set"].append(control)

        pm.parent(control, group)
        pm.parent(group, master_control)

        pm.move(control, [0, 0, offset], relative=True, objectSpace=True)
        pm.scale(control, [size, size, size])
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

        control = icon.create(
            name="{0}_main{1:0>2}_ctrl".format(prefix, joints.index(joint)),
            icon="sphere",
            color=[0, 1, 0]
        )
        control.setMatrix(group.getMatrix())
        pm.rotate(control, [0, 0, 90], relative=True, objectSpace=True)
        results["controls_set"].append(control)

        pm.parent(control, group)
        pm.parent(group, master_control)

        pm.move(control, [0, 0, offset], relative=True, objectSpace=True)
        pm.scale(control, [size, size, size])
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


class ui(MayaQWidgetDockableMixin, QtWidgets.QDialog):

    valueChanged = QtCore.Signal(int)

    def __init__(self, parent=None):
        super(ui, self).__init__(parent)

        self.filter = (
            "Shrinkwrap Rigger Configuration .shrinkwrap (*.shrinkwrap)"
        )

        self.create()

    def create(self):

        self.setWindowTitle("Shrinkwrap Rigger")
        self.setWindowFlags(QtCore.Qt.Window)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose, 1)

        self.create_layout()
        self.create_connections()

    def create_layout(self):

        main_layout = QtWidgets.QVBoxLayout()

        # Geometry input controls
        self.mesh_label = QtWidgets.QLabel("Mesh:")
        self.mesh = QtWidgets.QLineEdit()
        self.mesh_button = QtWidgets.QPushButton("<<")
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.mesh_label)
        layout.addWidget(self.mesh)
        layout.addWidget(self.mesh_button)
        main_layout.addLayout(layout)

        self.shrinkwrap_mesh_label = QtWidgets.QLabel("Shrinkwrap Mesh:")
        self.shrinkwrap_mesh = QtWidgets.QLineEdit()
        self.shrinkwrap_mesh_button = QtWidgets.QPushButton("<<")
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.shrinkwrap_mesh_label)
        layout.addWidget(self.shrinkwrap_mesh)
        layout.addWidget(self.shrinkwrap_mesh_button)
        main_layout.addLayout(layout)

        self.main_control_start_label = QtWidgets.QLabel("Main Control Start:")
        self.main_control_start = QtWidgets.QSpinBox()
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.main_control_start_label)
        layout.addWidget(self.main_control_start)
        main_layout.addLayout(layout)

        self.main_control_frequency_label = QtWidgets.QLabel(
            "Main Control Frequency:"
        )
        self.main_control_frequency = QtWidgets.QSpinBox()
        self.main_control_frequency.setMinimum(1)
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.main_control_frequency_label)
        layout.addWidget(self.main_control_frequency)
        main_layout.addLayout(layout)

        self.build_button = QtWidgets.QPushButton("Build")
        main_layout.addWidget(self.build_button)

        self.import_button = QtWidgets.QPushButton("Import Config From Json")
        main_layout.addWidget(self.import_button)

        self.export_button = QtWidgets.QPushButton("Export Config To Json")
        main_layout.addWidget(self.export_button)

        self.setLayout(main_layout)

    def create_connections(self):
        self.mesh_button.clicked.connect(
            partial(self.populate_object, self.mesh)
        )
        self.shrinkwrap_mesh_button.clicked.connect(
            partial(self.populate_object, self.shrinkwrap_mesh)
        )

        self.build_button.clicked.connect(self.build_rig)
        self.import_button.clicked.connect(self.import_settings)
        self.export_button.clicked.connect(self.export_settings)

    def populate_object(self, line_edit):
        selection = pm.selected()
        if selection:
            if len(selection) > 1:
                pm.displayWarning(
                    "Selected more and one object."
                    " Getting first selected object."
                )

            line_edit.setText(selection[0].name())
        else:
            pm.displayWarning("No object selected.")

    def build_rig(self):
        rig(**facial_rigger.lib.get_settings_from_widget(self))

    def export_settings(self):
        data_string = json.dumps(
            facial_rigger.lib.get_settings_from_widget(self),
            indent=4,
            sort_keys=True
        )

        file_path = facial_rigger.lib.get_file_path(self.filter, "save")
        if not file_path:
            return

        with open(file_path, "w") as f:
            f.write(data_string)

    def import_settings(self):
        file_path = facial_rigger.lib.get_file_path(self.filter, "open")
        if not file_path:
            return

        facial_rigger.lib.import_settings_from_file(file_path, self)


# Build from json file.
def rig_from_file(path):
    rig(**json.load(open(path)))


def show(*args):
    gqt.showDialog(ui)
