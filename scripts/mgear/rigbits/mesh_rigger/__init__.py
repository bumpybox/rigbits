from maya.app.general.mayaMixin import MayaQWidgetDockableMixin

from mgear.vendor.Qt import QtCore, QtWidgets
import mgear.core.pyqt as gqt
from . import shrinkwrap_rigger
from . import point_on_poly_rigger


class ui(MayaQWidgetDockableMixin, QtWidgets.QDialog):
    def __init__(self, parent=None):
        super(ui, self).__init__(parent)

        self.setWindowTitle("Mesh Rigger")
        self.setWindowFlags(QtCore.Qt.Window)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose, 1)

        tab_widget = QtWidgets.QTabWidget()

        dialogs = [shrinkwrap_rigger.ui(), point_on_poly_rigger.ui()]
        for dialog in dialogs:
            tab_widget.addTab(dialog, dialog.windowTitle())

        mainLayout = QtWidgets.QHBoxLayout()
        mainLayout.addWidget(tab_widget)
        self.setLayout(mainLayout)


def show(*args):
    gqt.showDialog(ui)


if __name__ == "__main__":
    show()
