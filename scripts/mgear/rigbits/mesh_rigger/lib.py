import operator

import pymel.core as pm


def create_edge_joint(edge, up_vector_position):
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

    # This gives us a vector that points from the center
    # of the selection to the highest vertex
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
