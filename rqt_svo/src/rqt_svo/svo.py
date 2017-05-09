#!/usr/bin/env python
import os
import rospy
import argparse
from qt_gui.plugin import Plugin
from .svo_widget import SvoWidget

class Svo(Plugin):
  """
  Subclass of Plugin to display SVO status
  """
  def __init__(self, context):
    
    # Init Plugin
    super(Svo, self).__init__(context)
    self.setObjectName('SvoPlugin')

    # Load arguments
    # TODO load topic name from args
    args = self._parse_args(context.argv())

    # Create QWidget
    self._widget = SvoWidget()
    
    # Show _widget.windowTitle on left-top of each plugin (when 
    # it's set in _widget). This is useful when you open multiple 
    # plugins at once. Also if you open multiple instances of your 
    # plugin at once, these lines add number to make it easy to 
    # tell from pane to pane.
    if context.serial_number() > 1:
        self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
    
    # Add widget to the user interface
    context.add_widget(self._widget)
    
  def _parse_args(self, argv):
    parser = argparse.ArgumentParser(prog='rqt_svo', add_help=False)
    group = parser.add_argument_group('Options for rqt_svo plugin')
    group.add_argument('topic', type=argparse.FileType('r'), nargs='*', default=[], help='Svo Info Topic to display')
    return parser.parse_args(argv)

  def save_settings(self, plugin_settings, instance_settings):
     # TODO save intrinsic configuration, usually using:
     # instance_settings.set_value(k, v)   
     print('Saving namespace')  
     namespace = self._widget._svo_namespace
     instance_settings.set_value('namespace', namespace)
     pass
# 
  def restore_settings(self, plugin_settings, instance_settings):
     # TODO restore intrinsic configuration, usually using:
     # v = instance_settings.value(k)
     namespace = instance_settings.value('namespace', 'default')
     self._widget.topic_line_edit.setText(namespace)
     pass

#   def shutdown_plugin(self):
#     # self._unregister_publisher()
#     pass
# 
#   def save_settings(self, plugin_settings, instance_settings):
#     # TODO save intrinsic configuration, usually using:
#     # instance_settings.set_value(k, v)
#     pass
# 
#   def restore_settings(self, plugin_settings, instance_settings):
#     # TODO restore intrinsic configuration, usually using:
#     # v = instance_settings.value(k)
#     pass


#     def _unregister_publisher(self):
#         if self._publisher is not None:
#             self._publisher.unregister()
#             self._publisher = None


  #def trigger_configuration(self):
      # Comment in to signal that the plugin has a way to configure
      # This will enable a setting button (gear icon) in each dock widget title bar
      # Usually used to open a modal configuration dialog
