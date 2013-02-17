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

import string
import sys
import threading
import time

import rosgraph
import roslib.message
import roslib.names
import rospy


class RosPlot2dException(Exception):
    pass


def _get_topic_type(topic):
    """
    subroutine for getting the topic type
    (nearly identical to rostopic._get_topic_type, except it returns rest of name instead of fn)

    :returns: topic type, real topic name, and rest of name referenced
      if the topic points to a field within a topic, e.g. /rosout/msg, ``str, str, str``
    """
    try:
        master = rosgraph.Master('/rosplot')
        val = master.getPublishedTopics('/')
    except:
        raise RosPlot2dException("unable to get list of topics from master")
    matches = [(t, t_type) for t, t_type in val if t == topic or topic.startswith(t + '/')]
    if matches:
        t, t_type = matches[0]
        if t_type == roslib.names.ANYTYPE:
            return None, None, None
        if t_type == topic:
            return t_type, None
        return t_type, t, topic[len(t):]
    else:
        return None, None, None


def get_topic_type(topic):
    """
    Get the topic type (nearly identical to rostopic.get_topic_type, except it doesn't return a fn)

    :returns: topic type, real topic name, and rest of name referenced
      if the \a topic points to a field within a topic, e.g. /rosout/msg, ``str, str, str``
    """
    topic_type, real_topic, rest = _get_topic_type(topic)
    if topic_type:
        return topic_type, real_topic, rest
    else:
        print >> sys.stderr, "WARNING: topic [%s] does not appear to be published yet. Waiting..." % topic
        while not rospy.is_shutdown():
            topic_type, real_topic, rest = _get_topic_type(topic)
            if topic_type:
                return topic_type, real_topic, rest
            else:
                time.sleep(0.1)
        return None, None, None


class ROSData2d(object):
    """
    Subscriber to ROS topic that buffers incoming data
    """

    def __init__(self, topicX, topicY, start_time):
        self.name = topicY + ' vs ' + topicX 
        self.start_time = start_time
        self.error = None

        self.lock = threading.Lock()
        self.buff_x = []
        self.buff_y = []
        self.buff_t = []

        self.last_x = 0
        self.last_y = 0
        self.last_t = start_time
        
        self._bEmptyX = True
        self._bEmptyY = True

        topic_typeX, real_topicX, fieldsX = get_topic_type(topicX)
        self.field_evalsX = generate_field_evals(fieldsX)
        data_classX = roslib.message.get_message_class(topic_typeX)
        self.subX = rospy.Subscriber(real_topicX, data_classX, self._ros_cbX)
        topic_typeY, real_topicY, fieldsY = get_topic_type(topicY)
        self.field_evalsY = generate_field_evals(fieldsY)
        data_classY = roslib.message.get_message_class(topic_typeY)
        self.subY = rospy.Subscriber(real_topicY, data_classY, self._ros_cbY)

    def close(self):
        self.subX.unregister()
        self.subY.unregister()

    def _ros_cb(self, msg, channel):
        """
        ROS subscriber callback
        :param msg: ROS message data
        """
        try:
            self.lock.acquire()
            try:
                if (not self._bEmptyX and not self._bEmptyY):
                    # record the starting point only if all three coordinates are valid
                    self.buff_x.append(self.last_x)
                    self.buff_y.append(self.last_y)
                    self.buff_t.append(self.last_t)
                
                if (channel == 'X'):
                    self.last_x = self._get_data(msg,channel)
                    self._bEmptyX = False
                elif (channel == 'Y'):
                    self.last_y = self._get_data(msg,channel)
                    self._bEmptyY = False
                    
                # #944: use message header time if present
                if msg.__class__._has_header:
                    self.last_t = (msg.header.stamp.to_sec() - self.start_time)
                else:
                    self.last_t = (rospy.get_time() - self.start_time)

                if (not self._bEmptyX and not self._bEmptyY):
                    # record the change
                    self.buff_x.append(self.last_x)
                    self.buff_y.append(self.last_y)
                    self.buff_t.append(self.last_t)
                    
            except AttributeError, e:
                self.error = RosPlot2dException("Invalid topic spec [%s]: %s" % (self.name, str(e)))
        finally:
            self.lock.release()
            
    def _ros_cbX(self, msg):
        """
        ROS subscriber callback
        :param msg: ROS message data
        """
        self._ros_cb(msg,'X')
                    
    def _ros_cbY(self, msg):
        """
        ROS subscriber callback
        :param msg: ROS message data
        """
        self._ros_cb(msg,'Y')
        
    def next(self):
        """
        Get the next data in the series

        :returns: [xdata], [ydata], [tdata]
        """
        if self.error:
            raise self.error
        try:
            self.lock.acquire()
            buffer_x = self.buff_x
            buffer_y = self.buff_y
            buffer_t = self.buff_t
            self.buff_x = []
            self.buff_y = []
            self.buff_t = []
        finally:
            self.lock.release()
        return buffer_x, buffer_y, buffer_t

    def _get_data(self, msg, channel):
        val = msg
        if (channel == 'X'):
            field_evals = self.field_evalsX
        elif (channel == 'Y'):
            field_evals = self.field_evalsY
        else:
            field_evals = False
        try:
            if not field_evals:
                return float(val)
            for f in field_evals:
                val = f(val)
            return float(val)
        except IndexError:
            self.error = RosPlot2dException("[%s] index error for: %s" % (self.name, str(val).replace('\n', ', ')))
        except TypeError:
            self.error = RosPlot2dException("[%s] value was not numeric: %s" % (self.name, val))


def _array_eval(field_name, slot_num):
    """
    :param field_name: name of field to index into, ``str``
    :param slot_num: index of slot to return, ``str``
    :returns: fn(msg_field)->msg_field[slot_num]
    """
    def fn(f):
        return getattr(f, field_name).__getitem__(slot_num)
    return fn


def _field_eval(field_name):
    """
    :param field_name: name of field to return, ``str``
    :returns: fn(msg_field)->msg_field.field_name
    """
    def fn(f):
        return getattr(f, field_name)
    return fn


def generate_field_evals(fields):
    try:
        evals = []
        fields = [f for f in fields.split('/') if f]
        for f in fields:
            if '[' in f:
                field_name, rest = f.split('[')
                slot_num = string.atoi(rest[:rest.find(']')])
                evals.append(_array_eval(field_name, slot_num))
            else:
                evals.append(_field_eval(f))
        return evals
    except Exception, e:
        raise RosPlot2dException("cannot parse field reference [%s]: %s" % (fields, str(e)))
