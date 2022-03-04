#! /usr/bin/env python
# -*- coding: utf-8 -*-#
# -------------------------------------------------------------------------------
# Name:         Open3dShowPcd
# Author:       shimingze
# Date:         2022/1/10 10:33
# Description:
# -------------------------------------------------------------------------------
from pathlib import Path
import random
import numpy as np
import open3d
from traceback import format_exc
import os
import struct
from msvcrt import getch
#---------------------------------------------------------------------------------------------------read-----------------------------------------------------------------------

def read_txt(txt_path,label_path = None):
    pcd = open3d.open3d.geometry.PointCloud()
    example = np.loadtxt(str(txt_path), delimiter=",", dtype=np.float32)[:, 0:3]
    # From numpy to Open3D
    pcd.points = open3d.open3d.utility.Vector3dVector(example)
    colorexample = [[1, 0, 0] for _ in range(len(example))]
    pcd.colors = open3d.open3d.utility.Vector3dVector(colorexample)
    print(colorexample)
    locs = []
    if label_path is not None:
        f = open(label_path, 'r')
        lines = f.readlines()
        for line in lines:
            line = line[:-1].split(",")
            textinfo = line[-7:]
            boxarr = []
            for txt in textinfo:
                boxarr.append(float(txt))
            # temp = boxarr[3]
            # boxarr[3] = boxarr[4]
            # boxarr[4] = temp
            locs.append(boxarr)
    return pcd, locs

def read_bin_velodyne(path):
    pc_list=[]
    with open(path,'rb') as f:
        content=f.read()
        pc_iter=struct.iter_unpack('ffff',content)
        for idx,point in enumerate(pc_iter):
            pc_list.append([point[0],point[1],point[2]])
    return np.asarray(pc_list,dtype=np.float32)

def read_bin(bin_path, label_path = None):
    pcd=open3d.open3d.geometry.PointCloud()
    example = read_bin_velodyne(bin_path)
    # From numpy to Open3D
    pcd.points= open3d.open3d.utility.Vector3dVector(example)
    label = np.fromfile(label_path,dtype=np.uint32).reshape((-1))
    print("label:--------------")
    colorexample = np.zeros([len(label),3])
    for cla in range(len(label)):
    #print(label[cla])
        if label[cla] == 40 or label[cla] == 44 or label[cla] == 48 or label[cla] == 49:
            colorexample[cla][0]=1
            colorexample[cla][1]=0
            colorexample[cla][2]=0
        else:
            colorexample[cla][0]=0
            colorexample[cla][1]=1
            colorexample[cla][2]=0
    print(colorexample)
    pcd.colors= open3d.open3d.utility.Vector3dVector(colorexample)

    locs = []
    # if label_path is not None:
    #     f = open(label_path,'r')
    #     lines = f.readlines()
    #     for line in lines:
    #         line = line.split(",")
    #         if len(line) and line[1] == "DontCare":
    #             continue
    #         textinfo = line[-7:]
    #         boxarr = []
    #         for txt in textinfo:
    #             boxarr.append(float(txt))
    #         #temp = boxarr[3]
    #         #boxarr[3] = boxarr[4]
    #         #boxarr[4] = temp
    #         locs.append(boxarr)
    return pcd, locs


#----------------------------------------------------------------------------------read--------------------------------------------------------

def rotz(t):
    """Rotation about the z-axis."""
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, -s,  0],
                     [s,  c,  0],
                     [0,  0,  1]])

def my_compute_box_3d(center, size, heading_angle):

    h = size[2]
    w = size[0]
    l = size[1]
    heading_angle = -heading_angle - np.pi / 2

    center[2] = center[2] + h / 2
    R = rotz(1*heading_angle)
    l = l/2
    w = w/2
    h = h/2
    x_corners = [-l,l,l,-l,-l,l,l,-l]
    y_corners = [w,w,-w,-w,w,w,-w,-w]
    z_corners = [h,h,h,h,-h,-h,-h,-h]
    corners_3d = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
    corners_3d[0,:] += center[0]
    corners_3d[1,:] += center[1]
    corners_3d[2,:] += center[2]
    return np.transpose(corners_3d)

class PtVis:
    def __init__(self, name='空格进入下一帧 Z退回上一帧', width=1024, height=768, file_dir=""):
        self.file_dir = file_dir
        self.pcd_path = file_dir+"/velodyne"
        self.label_path = file_dir+"/labels"
        self.pcd_files_list = []
        self.label_files_list = []
        self.index = 0
        self.pcd = None
        self.label = []
        self.vis = None
        self.name = name
        self.width = width
        self.height = height
        self.axis_pcd = open3d.geometry.TriangleMesh().create_coordinate_frame()
        self.init_pcd_files_list()
        self.init_label_files_list()
        self.real_locs = []
        # print(len(self.pcd_files_list))
        # self.pcd_totals = len(self.pcd_files_list)
    def init_setting(self):
        opt = self.vis.get_render_option()
        # 设置背景颜色和点大小
        opt.background_color = np.asarray([0, 0, 0])
        opt.point_size = 2



    def show_pcd(self):
        # 绘制open3d坐标系
        self.vis = open3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name=f"{Path(self.pcd_files_list[0]).name} {self.name}", width=self.width,
                               height=self.height)
        # 修改显示
        self.init_setting()
        # 初始化显示pcd第一帧
        self.pcd,self.real_locs = read_bin(self.pcd_files_list[0],self.label_files_list[0])
        # self.pcd = o3d.io.read_point_cloud(self.pcd_files_list[0])
        self.vis.add_geometry(self.pcd)
        # self.vis.add_geometry(self.axis_pcd)
        # 设置键盘响应事件
        random.seed(10)
        for bbox in self.real_locs:
            corners_3d = my_compute_box_3d(bbox[0:3], bbox[3:6], bbox[6])
            bbox_lines = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6],
                          [3, 7]]
            colors = [[0, 1, 0] for _ in range(len(bbox_lines))]  # green
            bbox = open3d.geometry.LineSet()
            bbox.lines = open3d.utility.Vector2iVector(bbox_lines)
            bbox.colors = open3d.utility.Vector3dVector(colors)
            bbox.points = open3d.utility.Vector3dVector(corners_3d)
            self.vis.add_geometry(bbox)
        self.vis.register_key_callback(90, lambda temp: self.last())
        self.vis.register_key_callback(32, lambda temp: self.next())
        self.vis.run()

    def next(self):
        if 0 <= self.index <= len(self.pcd_files_list)-1:
            self.update(self.index)
            self.index += 1
    def last(self):
        if 0 < self.index <= len(self.pcd_files_list) - 1:
            self.index -= 1
            self.update(self.index)
    def init_pcd_files_list(self):
        for file in Path(self.pcd_path).rglob("*.bin"):
            self.pcd_files_list.append(str(file))
        self.pcd_files_list = sorted(self.pcd_files_list, key=lambda p: Path(p).stem)

    def init_label_files_list(self):
        for file in Path(self.label_path).rglob("*.label"):
            self.label_files_list.append(str(file))
        self.label_files_list = sorted(self.label_files_list, key=lambda p: Path(p).stem)
    def update(self, index):
        new_pcd_file = self.pcd_files_list[index]
        self.vis.create_window(window_name=f"{Path(new_pcd_file).name} {self.name}", width=self.width,
                               height=self.height)
        self.pcd,self.real_locs = read_bin(self.pcd_files_list[index],self.label_files_list[index])

        self.vis.clear_geometries()  # 清空vis点云
        for bbox in self.real_locs:
            corners_3d = my_compute_box_3d(bbox[0:3], bbox[3:6], bbox[6])
            bbox_lines = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6],
                          [3, 7]]
            colors = [[0, 1, 0] for _ in range(len(bbox_lines))]  # green
            bbox = open3d.geometry.LineSet()
            bbox.lines = open3d.utility.Vector2iVector(bbox_lines)
            bbox.colors = open3d.utility.Vector3dVector(colors)
            bbox.points = open3d.utility.Vector3dVector(corners_3d)
            self.vis.add_geometry(bbox)
        # self.vis.vis.update_geometry(bbox)
        self.vis.add_geometry(self.pcd)  # 增加vis中的点云
        # self.vis.add_geometry(self.axis_pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

    def close(self):
        self.vis.destroy_window()

if __name__ == '__main__':
    pt = None
    try:
        # pcd_file_path = input("请输入要展示的pcd文件夹:").strip("\"")
        file_dir = r"E:\dataset\dataset\sequences\00"
        # H:\pyworkplace\smz\Cat3DVisualTools\\1641460079.483406\pcd
        # pcd_file_path = r"C:\Users\pc\Desktop\docker\output\result"
        pt = PtVis(file_dir=file_dir)
        # print(f"{pcd_file_path}共有PCD个数:{len(pt.pcd_files_list)}")
        pt.show_pcd()
    except Exception as e:
        print(f"运行出错请联系开发人员:{format_exc}{e}")
    finally:
        pt.close()