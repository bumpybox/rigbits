import operator
from functools import partial
import json

import pymel.core as pm
from maya.app.general.mayaMixin import MayaQWidgetDockableMixin

from mgear.vendor.Qt import QtWidgets, QtCore
from mgear.rigbits import facial_rigger


def create_edge_joint(edge, up_vector_position, up_vector_highest=False):
    # Get the vertices the edge is connected to
    edgeVerts = edge.connectedVertices()

    # Cluster the verts.
    # We will use this to get the position for our joint
    clusTfm = pm.cluster(edgeVerts)[1]

    pm.select(clear=True)
    # Create our joint
    jnt = pm.joint()

    # getPosition doesn't give us the correct result. This does
    pos = clusTfm.rotatePivot.get()
    # We don't need the cluster any more
    pm.delete(clusTfm)

    # Now we calculate the average normal
    normals = []
    for face in edge.connectedFaces():
        # Collect the normal of every face
        normals.append(face.getNormal(space="world"))

    # Variable that will store the sum of all normals
    normalsSum = pm.datatypes.Vector()
    for normal in normals:
        normalsSum += normal

    # This will be our vector for the x axis
    # Average normal.
    # We divide the normal by the total number of vectors
    xVec = (normalsSum / len(normals))

    # The vertex that has the highest position,
    # will be the vertex that our Y axis will point to
    upVec = None
    for i, vert in enumerate(edgeVerts):
        # We take the first vert as our up vector
        if i == 0:
            upVec = edgeVerts[0].getPosition(space="world")

        # And compare the other to it
        vertPos = edgeVerts[i].getPosition(space="world")
        if vertPos[1] >= upVec[1]:
            upVec = vertPos

    # This gives us a vector that points from the center
    # of the selection to the highest vertex
    if up_vector_highest:
        up_vector_position = upVec - pos
    else:
        up_vector_position = up_vector_position - pos

    # We get the z vector from the cross product of our x vector
    # and the up vector
    zVec = xVec.cross(up_vector_position)
    # Calculate the y vec the same way. We could use the up_vector_position
    # but this way we make sure they are all correct
    yVec = zVec.cross(xVec)

    # Normalize all vectors so scaling doesn't get messed up
    xVec.normalize()
    yVec.normalize()
    zVec.normalize()

    # Construct the matrix from the vectors we calculated
    jntMtx = pm.dt.Matrix(-zVec, yVec, xVec, pos)
    # And set the joints matrix to our new matrix
    jnt.setMatrix(jntMtx)

    # This transfers the rotation values
    # to the joint orientation values.
    pm.makeIdentity(jnt, r=True, apply=True)

    return jnt


def connected_verts(connected_verts, results=[]):
    next_verts = list(
        set(results[-1].connectedVertices()) & set(connected_verts)
    )
    for vert in next_verts:
        if vert not in results:
            results.append(vert)
            break

    return results


def extract_organization_keys(data):
    keys = [
        "setup_group",
        "deformers_group",
        "deformers_set",
        "controls_group",
        "controls_set"
    ]
    results = {}
    for key in keys:
        results[key] = data.get(key)
        if key in data.keys():
            data.pop(key)

    return data, results


def organize_results(data, **kwargs):
    for key, value in kwargs.iteritems():
        if value is None:
            continue

        node = pm.PyNode(value)

        if node.nodeType() == "transform":
            for child in data[key]:
                pm.parent(child, node)

        if node.nodeType() == "objectSet":
            node.addMembers(data[key])


class rename_object(object):

    def __init__(self, node):
        self.node = node
        self.x = node.getTranslation(space="world")[0]
        self.y = node.getTranslation(space="world")[1]
        self.name = ""


def rename_by_position(nodes, tolerance=0.001, prefix="", suffix=""):
    """Rename nodes based on position.

    Finds a unique name by indexing in x axis.
    """
    nodes = [pm.PyNode(x) for x in nodes]

    objects = []
    for node in nodes:
        objects.append(rename_object(node))

    # Sort by y axis, top to bottom
    objects.sort(key=operator.attrgetter("y"))
    objects.reverse()

    # Get positional pairs
    objects_copy = list(objects)
    position_pairs = []
    for count in range(0, len(objects_copy)):
        a = objects_copy[0]
        del(objects_copy[0])
        for b in objects_copy:
            if abs(a.y - b.y) <= tolerance:
                position_pairs.append([a, b])

    # Name positional pairs
    for pair in position_pairs:
        # Sort pairs by x value.
        pair.sort(key=operator.attrgetter("x"))
        index = position_pairs.index(pair) + 1
        pair[0].name = "{0}{1:0>2}".format("Rt", index)
        pair[1].name = "{0}{1:0>2}".format("Lt", index)

    # Name nodes
    center_count = 1
    for object in objects:
        # Non positional pairs will be center "C"
        if not object.name:
            object.name = "{0}{1:0>2}".format("C", center_count)
            center_count += 1

        if prefix:
            object.name = prefix + object.name

        if suffix:
            object.name = object.name + suffix

        pm.rename(object.node, object.name)


class settings_dialog(MayaQWidgetDockableMixin, QtWidgets.QDialog):

    def __init__(self, parent=None):
        super(settings_dialog, self).__init__(parent)

        self.setWindowFlags(QtCore.Qt.Window)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose, 1)

        self.main_layout = QtWidgets.QVBoxLayout()
        self.setLayout(self.main_layout)

        self.create_header_layout()
        self.create_body_layout()
        self.create_footer_layout()

    def create_header_layout(self):

        # prefix
        self.prefix_label = QtWidgets.QLabel("Prefix:")
        self.prefix = QtWidgets.QLineEdit()
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.prefix_label)
        layout.addWidget(self.prefix)
        self.main_layout.addLayout(layout)

        # control_size
        self.control_size_label = QtWidgets.QLabel("Control Size:")
        self.control_size = QtWidgets.QDoubleSpinBox()
        self.control_size.setValue(1)
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.control_size_label)
        layout.addWidget(self.control_size)
        self.main_layout.addLayout(layout)

        # control_offset
        self.control_offset_label = QtWidgets.QLabel("Control Offset:")
        self.control_offset = QtWidgets.QDoubleSpinBox()
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.control_offset_label)
        layout.addWidget(self.control_offset)
        self.main_layout.addLayout(layout)

        # setup_group
        self.setup_group_label = QtWidgets.QLabel("Setup Group:")
        self.setup_group = QtWidgets.QLineEdit()
        self.setup_group_button = QtWidgets.QPushButton("<<")
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.setup_group_label)
        layout.addWidget(self.setup_group)
        layout.addWidget(self.setup_group_button)
        self.main_layout.addLayout(layout)

        self.setup_group_button.clicked.connect(
            partial(self.populate_object, self.setup_group)
        )

        # controls_group
        self.controls_group_label = QtWidgets.QLabel("Controls Group:")
        self.controls_group = QtWidgets.QLineEdit()
        self.controls_group_button = QtWidgets.QPushButton("<<")
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.controls_group_label)
        layout.addWidget(self.controls_group)
        layout.addWidget(self.controls_group_button)
        self.main_layout.addLayout(layout)

        self.controls_group_button.clicked.connect(
            partial(self.populate_object, self.controls_group)
        )

        # controls_set
        self.controls_set_label = QtWidgets.QLabel("Controls Set:")
        self.controls_set = QtWidgets.QLineEdit()
        self.controls_set_button = QtWidgets.QPushButton("<<")
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.controls_set_label)
        layout.addWidget(self.controls_set)
        layout.addWidget(self.controls_set_button)
        self.main_layout.addLayout(layout)

        self.controls_set_button.clicked.connect(
            partial(self.populate_object, self.controls_set)
        )

    def create_body_layout(self):
        pass

    def create_footer_layout(self):
        self.build_button = QtWidgets.QPushButton("Build")
        self.main_layout.addWidget(self.build_button)
        self.build_button.clicked.connect(self.build_rig)

        self.import_button = QtWidgets.QPushButton("Import Config From Json")
        self.main_layout.addWidget(self.import_button)
        self.import_button.clicked.connect(self.import_settings)

        self.export_button = QtWidgets.QPushButton("Export Config To Json")
        self.main_layout.addWidget(self.export_button)
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

    def populate_objects(self, line_edit):
        selection = pm.selected(flatten=True)
        if selection:
            line_edit.setText(",".join([node.name() for node in selection]))
        else:
            pm.displayWarning("No objects selected.")

    def build_rig(self):
        pass

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
