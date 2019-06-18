#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
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
#  * Neither the name of TUM nor the names of its
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
# the resulting .ply file can be viewed for example with meshlab
# sudo apt-get install meshlab

"""
This script reads a registered pair of color and depth images and generates a
colored 3D point cloud in the PLY format.
"""

import argparse
import sys
import os
from open3d import *
from PIL import Image
import numpy as np
from multiprocessing import Process
from shutil import copyfile
import cv2
import time

focalLength = 525.0
centerX = 319.5
centerY = 239.5
scalingFactor = 5000.0

def show_pcl(rgb_file,depth_file):
    vis = Visualizer()
    vis.create_window()
    pcd = PointCloud()
    geo_flag = False
    while True:
        
        pcd.clear()
        #pcd = read_point_cloud(ply_file)
        pts,clr,err_flag = generate_pointcloud(rgb_file,depth_file)
        if err_flag is not None:
            continue
       
        #ply_load = read_point_cloud(ply_file)
        pcd.points = Vector3dVector(pts)
        if not geo_flag:
            vis.add_geometry(pcd)
            geo_flag = True        
        
        #pcd.points = ply_load.points
        #pcd.colors = ply_load.colors
 
        # Visualizing lidar points in camera coordinates

        n = pts.shape[0]
        s = 0
        for idx in range(0,n,int(n/100)): 
        # Visualizing lidar points in camera coordinates
            pcd.points = Vector3dVector(pts[0:idx,:])
            pcd.colors = Vector3dVector(clr[0:idx,:])
            vis.update_geometry()
            vis.poll_events()
            vis.update_renderer()
            s = s+idx
        for i in range(0,n-s):
            pcd.points = Vector3dVector(pts[0:i,:])
            pcd.colors = Vector3dVector(clr[0:idx,:])
            vis.update_geometry()
            vis.poll_events()
            vis.update_renderer()
 
def generate_pointcloud(rgb_file,depth_file):
    """
    Generate a colored point cloud in PLY format from a color and a depth image.
    
    Input:
    rgb_file -- filename of color image
    depth_file -- filename of depth image
    ply_file -- filename of ply file
    
    """
    try:
        rgb = Image.open(rgb_file)
        depth = Image.open(depth_file)
    
    
        if rgb.size != depth.size:
            raise Exception("Color and depth image do not have the same resolution.")
        if rgb.mode != "RGB":
            raise Exception("Color image is not in RGB format")
        if depth.mode != "I":
            raise Exception("Depth image is not in intensity format")

        points = []
        data_xyz = []
        color_map = []    
        for v in range(rgb.size[1]):
            for u in range(rgb.size[0]):
                color = rgb.getpixel((u,v))
                Z = depth.getpixel((u,v)) / scalingFactor
                if Z==0: continue
                X = (u - centerX) * Z / focalLength
                Y = (v - centerY) * Z / focalLength
                points.append("%f %f %f %d %d %d 0\n"%(X,Y,Z,color[0],color[1],color[2]))
                data_xyz.append(float(X))
                data_xyz.append(float(Y))
                data_xyz.append(float(Z))
                data_xyz.append(float(color[0]/255.0))
                data_xyz.append(float(color[1]/255.0))
                data_xyz.append(float(color[2]/255.0))
    except:
        return None,None,"set"

    data_xyz = np.asarray(data_xyz)
    data_xyz =data_xyz.reshape(-1,6)
    data = []
    color_map = data_xyz[:,3:]
    data = data_xyz[:,:3]
    return data,color_map,None
    

if __name__ == '__main__':
    rgb_file = "../data/live_feed/image_01/data/image.png"
    depth_file = "../data/live_feed/depth_completed/depth.png"
    #generate_pointcloud(rgb_file,depth_file)
    show_pcl(rgb_file,depth_file)
    
    
