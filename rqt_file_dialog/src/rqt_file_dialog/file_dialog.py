import os
import rospkg
import rospy

from python_qt_binding import QtCore
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget
from python_qt_binding.QtWidgets import QFileDialog, QLineEdit, QPushButton
from qt_gui.plugin import Plugin
from std_msgs.msg import String


class RqtFileDialog(Plugin):

    def __init__(self, context):
        super(RqtFileDialog, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('RqtFileDialog')
        rp = rospkg.RosPack()

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('rqt_file_dialog'), 'resource', 'file_dialog.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('RqtFileDialogUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self.select_button = self._widget.findChild(QPushButton, 'select_button')
        self.select_button.clicked.connect(self.handle_select)
        self.current_line_edit = self._widget.findChild(QLineEdit, 'current_line_edit')
        self.current_line_edit.editingFinished.connect(self.publish)

        self.pub = None

    def handle_select(self):
        # TODO(lucasw) have a parameter define which kind of dialog to use
        file_dir = self.current_line_edit.text()
        new_file_dir = None
        if self.mode == "file_save":
            new_file_dir, tmp = QFileDialog.getSaveFileName(caption="Select a save file",
                                                            directory=os.path.dirname(file_dir))
        elif self.mode == "file_open":
            new_file_dir, tmp = QFileDialog.getOpenFileName(caption="Select a file",
                                                            directory=os.path.dirname(file_dir))
        else:  # elif mode == "dir":
            new_file_dir = QFileDialog.getExistingDirectory(caption="Select a directory",
                                                            directory=os.path.dirname(file_dir))
        if new_file_dir is not None and new_file_dir != "":
            self.current_line_edit.setText(new_file_dir)
            self.publish()

    def publish(self):
        if self.pub is not None:
            self.pub.publish(self.current_line_edit.text())

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value("file_dir", self.current_line_edit.text())
        instance_settings.set_value("mode", self.mode)
        instance_settings.set_value("select_text", self.select_button.text())
        if self.pub:
            instance_settings.set_value("topic", self.topic)

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.contains("file_dir"):
            file_dir = instance_settings.value("file_dir")
            self.current_line_edit.setText(file_dir)
            self.publish()
        self.mode = "file_save"
        if instance_settings.contains("mode"):
            self.mode = instance_settings.value("mode")
        self.topic = "file_dir"
        if instance_settings.contains("topic"):
            self.topic = instance_settings.value("topic")
        self.pub = rospy.Publisher(self.topic, String, queue_size=1, latch=True)

        if instance_settings.contains("select_text"):
            self.select_button.setText(instance_settings.value("select_text"))

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
