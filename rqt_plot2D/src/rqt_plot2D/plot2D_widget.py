#!/usr/bin/env python

# File created by David Dovrat, 2013.
# Derived from http://ros.org/wiki/rqt_plot. Thanks go to Dorian Scholz, TU Darmstadt and Willow Garage Inc. For providing the infrastructure for all the rest of us.
# Check out the license agreement:
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import os
import roslib
roslib.load_manifest('rqt_plot2D')

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QAction, QIcon, QMenu, QWidget

import rospy

from rqt_py_common.topic_completer import TopicCompleter
from rqt_py_common.topic_helpers import is_slot_numeric

from .rosplot2d import ROSData2d, RosPlot2dException


class Plot2dWidget(QWidget):
    _redraw_interval = 40

    def __init__(self, arguments=None, initial_topics=None):
        super(Plot2dWidget, self).__init__()
        self.setObjectName('Plot2dWidget')

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'plot2D.ui')
        loadUi(ui_file, self)
        self.subscribe_topic_button.setIcon(QIcon.fromTheme('add'))
        self.remove_topic_button.setIcon(QIcon.fromTheme('remove'))
        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.clear_button.setIcon(QIcon.fromTheme('edit-clear'))
        self.data_plot = None
        self.nTopicCounter = 0
        self.topicNames = []
        self.bTopicValidA = False
        self.bTopicValidB = False

        self.subscribe_topic_button.setEnabled(False)

        self._topic_completerA = TopicCompleter(self.topicEditA)
        self._topic_completerB = TopicCompleter(self.topicEditB)
        self._topic_completerA.update_topics()
        self._topic_completerB.update_topics()
        self.topicEditA.setCompleter(self._topic_completerA)
        self.topicEditB.setCompleter(self._topic_completerB)

        self._start_time = rospy.get_time()
        self._rosdata = {}
        self._remove_topic_menu = QMenu()

        # init and start update timer for plot
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)

        # save command line arguments
        self._arguments = arguments
        self._initial_topics = initial_topics

    def switch_data_plot_widget(self, data_plot):
        self.enable_timer(enabled=False)

        self.data_plot_layout.removeWidget(self.data_plot)
        if self.data_plot is not None:
            self.data_plot.close()

        self.data_plot = data_plot
        self.data_plot_layout.addWidget(self.data_plot)

        # setup drag 'n drop
        self.data_plot.dropEvent = self.dropEvent
        self.data_plot.dragEnterEvent = self.dragEnterEvent

        if self._initial_topics:
            for topic_name in self._initial_topics:
                self.add_topic(topic_name)
            self._initial_topics = None
        else:
            for topic_name, rosdata in self._rosdata.items():
                data_x, data_y, data_t = rosdata.next()
                self.data_plot.add_curve(topic_name, topic_name, data_x, data_y)

        self._subscribed_topics_changed()

    @Slot(str)
    def on_topicEditA_textChanged(self, topic_name):
        # on empty topic name, update topics
        if topic_name in ('', '/'):
            self._topic_completerA.update_topics()

        is_numeric, is_array, message = is_slot_numeric(topic_name)
        self.bTopicValidA = (is_numeric and not is_array)
        bSubscribeEnabled = (self.bTopicValidA and self.bTopicValidB)
        if (self.bTopicValidA and not bSubscribeEnabled):
            message = 'Please insert a valid topic to slot B'
        self.subscribe_topic_button.setEnabled(bSubscribeEnabled)
        self.subscribe_topic_button.setEnabled(bSubscribeEnabled)
        self.subscribe_topic_button.setToolTip(message)

    @Slot(str)
    def on_topicEditB_textChanged(self, topic_name):
        # on empty topic name, update topics
        if topic_name in ('', '/'):
            self._topic_completerB.update_topics()

        is_numeric, is_array, message = is_slot_numeric(topic_name)
        self.bTopicValidB = (is_numeric and not is_array)
        bSubscribeEnabled = (self.bTopicValidA and self.bTopicValidB)
        if (self.bTopicValidB and not bSubscribeEnabled):
            message = 'Please insert a valid topic to slot A'
        self.subscribe_topic_button.setEnabled(bSubscribeEnabled)
        self.subscribe_topic_button.setToolTip(message)
        
    @Slot()
    def on_subscribe_topic_button_clicked(self):
        self.add_topic(str(self.topicEditB.text()) + ' vs ' + str(self.topicEditA.text()))

    @Slot(bool)
    def on_pause_button_clicked(self, checked):
        self.enable_timer(not checked)

    @Slot()
    def on_clear_button_clicked(self):
        self.clean_up_subscribers()

    def update_plot(self):
        for topic_name, rosdata in self._rosdata.items():
            try:
                data_x, data_y, data_t = rosdata.next()
                self.data_plot.update_values(topic_name, data_x, data_y)
            except RosPlot2dException as e:
                qWarning('PlotWidget.update_plot(): error in rosplot: %s' % e)
        self.data_plot.redraw()
        return
    
    def _subscribed_topics_changed(self):
        self._update_remove_topic_menu()
        if self._arguments:
            if self._arguments.start_paused:
                self.pause_button.setChecked(True)
        if not self.pause_button.isChecked():
            # if pause button is not pressed, enable timer based on subscribed topics
            self.enable_timer(self._rosdata)

    def _update_remove_topic_menu(self):
        def make_remove_topic_function(x):
            return lambda: self.remove_topic(x)

        self._remove_topic_menu.clear()
        for topic_name in sorted(self._rosdata.keys()):
            action = QAction(topic_name, self._remove_topic_menu)
            action.triggered.connect(make_remove_topic_function(topic_name))
            self._remove_topic_menu.addAction(action)

        self.remove_topic_button.setMenu(self._remove_topic_menu)

    def add_topic(self, topic_name):
        if topic_name in self._rosdata:
            qWarning('PlotWidget.add_topic(): topic already subscribed: %s' % topic_name)
        else:
            self.topicNames.append(topic_name)
            self._rosdata[topic_name] = ROSData2d(str(self.topicEditA.text()), str(self.topicEditB.text()), self._start_time)
            data_x, data_y, data_t = self._rosdata[topic_name].next()
            self.data_plot.add_curve(topic_name, topic_name, data_x, data_y)
            self._subscribed_topics_changed()
            return  

    def remove_topic(self, topic_name):
        self._rosdata[topic_name].close()
        del self._rosdata[topic_name]
        self.data_plot.remove_curve(topic_name)

        self._subscribed_topics_changed()

    def clean_up_subscribers(self):
        for topic_name, rosdata in self._rosdata.items():
            rosdata.close()
            self.data_plot.remove_curve(topic_name)
        self._rosdata = {}

        self._subscribed_topics_changed()

    def enable_timer(self, enabled=True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()