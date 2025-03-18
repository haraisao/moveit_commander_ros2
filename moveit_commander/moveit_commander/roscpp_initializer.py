# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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
# Author: Ioan Sucan

import os
from tempfile import NamedTemporaryFile
import yaml
import glob
import subprocess
from ament_index_python.packages import get_package_share_directory
import sys

from moveit_ros_planning_interface_py import _moveit_roscpp_initializer

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def load_kinematics(name):
    return load_yaml(name+'_moveit_config', 'config/kinematics.yaml')

def dump_param_file(params, delete=False):
    with NamedTemporaryFile(mode="w", prefix="wrap_moveit_param_", delete=delete) as f:
        param_file_ = f.name
        param_dict = { '/**': {'ros__parameters': params } }
        yaml.dump(param_dict, f, default_flow_style=False)
        return param_file_

def get_robot_semantic():
    res=subprocess.run(["ros2", "param", "get", "/move_group", "robot_description_semantic"], capture_output=True)
    return res.stdout.decode()

def gen_srdf_param():
   srdf=get_robot_semantic()
   fname=dump_param_file({"robot_description_semantic": srdf[17:]})
   return ["--params-file", fname]

def gen_kinematics_param(name):
   kinematics=load_kinematics(name)
   fname=dump_param_file(kinematics)
   return ["--params-file", fname]

def clear_tempfiles(name="wrap_moveit_param_"):
    flist=glob.glob("/tmp/"+name+"*")
    for fn in flist:
      os.remove(fn)

def roscpp_initialize(args=sys.argv, robot_name="", use_sim=False):
    get_robot_semantic()
    if len(args) > 1:
      # remove __name:= argument
      #args2 = [a for a in args if not a.startswith("__name:=")]
      args2=[]
      for a in args:
        if a:
          if a.startswith("__name:="):
            args2.append(a+"_wrap_python")
          else:
            args2.append(a)
    else:
      args2=["--ros-args", "-p", "use_sim_time:="+str(use_sim)] + gen_srdf_param()
      if robot_name:
         args2 += gen_kinematics_param(robot_name)

      args2 = [a for a in args2 if a]
    _moveit_roscpp_initializer.roscpp_init("move_group_commander_wrappers", args2)

def roscpp_shutdown():
    _moveit_roscpp_initializer.roscpp_shutdown()
