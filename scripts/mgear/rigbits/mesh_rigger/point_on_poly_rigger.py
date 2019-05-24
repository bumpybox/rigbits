from functools import partial
import json

import pymel.core as pm

from mgear.core import icon
from . import lib
from mgear.vendor.Qt import QtWidgets
from mgear.rigbits import facial_rigger
import mgear.core.pyqt as gqt


def rig(*args, **kwargs):
    kwargs, organization_keys = lib.extract_organization_keys(kwargs)
    results = _rig(*args, **kwargs)
    lib.organize_results(results, **organization_keys)
    lib.rename_by_position(
        results["controls_set"], prefix=kwargs["prefix"] + "_", suffix="_ctrl"
    )


def _rig(verts=[],
         look_at_verts=[],
         prefix="pop_rig",
         control_size=1.0,
         control_offset=0.0):

    results = {
        "setup_group": [],
        "deformers_group": [],
        "deformers_set": [],
        "controls_group": [],
        "controls_set": []
    }

    verts = [pm.PyNode(x) for x in verts]
    look_at_verts = [pm.PyNode(x) for x in look_at_verts]

    ordered_verts = [verts[0]]
    for count in range(0, len(verts)):
        ordered_verts = lib.connected_verts(verts, ordered_verts)

    geo = pm.listRelatives(ordered_verts[0], parent=True)[0]

    for vert in ordered_verts:
        normal_group = pm.group(
            name="{0}_normal{1:0>2}_grp".format(
                prefix, ordered_verts.index(vert)
            ),
            empty=True
        )
        pm.select(vert, normal_group)
        pm.runtime.PointOnPolyConstraint()

        # Break rotation connections
        for attr in ["rx", "ry", "rz"]:
            pm.disconnectAttr("{0}.{1}".format(normal_group, attr))

        pm.normalConstraint(geo, normal_group)

        # Up vector group
        up_vector_group = pm.group(
            name="{0}_up_vector{1:0>2}_grp".format(
                prefix, ordered_verts.index(vert)
            ),
            empty=True
        )
        up_vector_group.setMatrix(normal_group.getMatrix())
        pm.parent(up_vector_group, normal_group)
        up_vector_group.tx.set(0.001)

        # Look at group
        look_at_vert = None
        for connected_vert in vert.connectedVertices():
            if connected_vert in look_at_verts:
                look_at_vert = connected_vert
        print(look_at_vert)
        print(vert)
        look_at_group = pm.group(
            name="{0}_look_at{1:0>2}_grp".format(
                prefix, ordered_verts.index(vert)
            ),
            empty=True
        )
        pm.select(look_at_vert, look_at_group)
        pm.runtime.PointOnPolyConstraint()

        # Parent to setup group
        results["setup_group"].append(normal_group)
        results["setup_group"].append(look_at_group)

        # Transform group
        transform_group = pm.group(
            name="{0}_transform{1:0>2}_grp".format(
                prefix, ordered_verts.index(vert)
            ),
            empty=True
        )
        transform_group.setMatrix(normal_group.getMatrix())
        pm.aimConstraint(
            look_at_group,
            transform_group,
            aimVector=[0, 1, 0],
            upVector=[0, 0, 1],
            worldUpObject=up_vector_group,
            worldUpType="object"
        )
        pm.parent(transform_group, normal_group)

        # Control
        parent_group = pm.group(
            name="{0}_parent{1:0>2}_grp".format(
                prefix, ordered_verts.index(vert)
            ),
            empty=True
        )
        pm.parentConstraint(transform_group, parent_group)
        results["controls_group"].append(parent_group)

        control = icon.create(
            name="{0}_pop{1:0>2}_ctrl".format(
                prefix, ordered_verts.index(vert)
            ),
            icon="sphere",
            color=[0, 0, 1]
        )
        control.setMatrix(parent_group.getMatrix())
        pm.parent(control, parent_group)

        pm.scale(control, [control_size, control_size, control_size])
        pm.move(
            control, [0, 0, control_offset], relative=True, objectSpace=True
        )
        pm.makeIdentity(control, apply=True)
        control.resetFromRestPosition()

        results["controls_set"].append(control)

        # Joint
        pm.select(clear=True)
        joint = pm.joint(
            name="{0}_pop{1:0>2}_jnt".format(prefix, ordered_verts.index(vert))
        )
        pm.parentConstraint(control, joint)

        results["deformers_group"].append(joint)
        results["deformers_set"].append(joint)

    return results


class ui(lib.settings_dialog):

    def __init__(self, parent=None):
        super(ui, self).__init__(parent)

        self.filter = (
            "Point-on-Poly Rigger Configuration .pop (*.pop)"
        )

        self.setWindowTitle("Point-on-Poly Rigger")

    def create_body_layout(self):

        # verts
        self.verts_label = QtWidgets.QLabel("Verts:")
        self.verts = QtWidgets.QLineEdit()
        self.verts_button = QtWidgets.QPushButton("<<")
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.verts_label)
        layout.addWidget(self.verts)
        layout.addWidget(self.verts_button)
        self.main_layout.addLayout(layout)

        self.verts_button.clicked.connect(
            partial(self.populate_objects, self.verts)
        )

        # verts
        self.look_at_verts_label = QtWidgets.QLabel("Look At Verts:")
        self.look_at_verts = QtWidgets.QLineEdit()
        self.look_at_verts_button = QtWidgets.QPushButton("<<")
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.look_at_verts_label)
        layout.addWidget(self.look_at_verts)
        layout.addWidget(self.look_at_verts_button)
        self.main_layout.addLayout(layout)

        self.look_at_verts_button.clicked.connect(
            partial(self.populate_objects, self.look_at_verts)
        )

        # deformers_group
        self.deformers_group_label = QtWidgets.QLabel("Deformers Group:")
        self.deformers_group = QtWidgets.QLineEdit()
        self.deformers_group_button = QtWidgets.QPushButton("<<")
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.deformers_group_label)
        layout.addWidget(self.deformers_group)
        layout.addWidget(self.deformers_group_button)
        self.main_layout.addLayout(layout)

        self.deformers_group_button.clicked.connect(
            partial(self.populate_object, self.deformers_group)
        )

        # deformers_set
        self.deformers_set_label = QtWidgets.QLabel("Deformers Set:")
        self.deformers_set = QtWidgets.QLineEdit()
        self.deformers_set_button = QtWidgets.QPushButton("<<")
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.deformers_set_label)
        layout.addWidget(self.deformers_set)
        layout.addWidget(self.deformers_set_button)
        self.main_layout.addLayout(layout)

        self.deformers_set_button.clicked.connect(
            partial(self.populate_object, self.deformers_set)
        )

    def build_rig(self):
        kwargs = facial_rigger.lib.get_settings_from_widget(self)
        kwargs["verts"] = kwargs["verts"].split(",")
        kwargs["look_at_verts"] = kwargs["look_at_verts"].split(",")
        rig(**kwargs)


# Build from json file.
def rig_from_file(path):
    kwargs = json.load(open(path))
    kwargs["verts"] = kwargs["verts"].split(",")
    kwargs["look_at_verts"] = kwargs["look_at_verts"].split(",")
    rig(**kwargs)


def show(*args):
    gqt.showDialog(ui)
