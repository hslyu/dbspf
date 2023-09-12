#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr

import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.proj3d import proj_transform
from mpl_toolkits.mplot3d.axes3d import Axes3D
from matplotlib.text import Annotation

# Import parent directory
import os,sys,inspect
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
from utils import open_json

idx=3
default_path = f'/storage/hslyu/ours-user-rate5-tw4/user_80/depth_5/env_{idx:04d}-depth_5-ue_80.json'

class Annotation3D(Annotation):	#{{{
    '''
    https://gist.github.com/hslyu/16b590fc12dfbf455a323a780053eb95
    '''
    def __init__(self, text, xyz, *args, **kwargs):
        super().__init__(text, xy=(0, 0), *args, **kwargs)
        self._xyz = xyz

    def draw(self, renderer):
        x2, y2, z2 = proj_transform(*self._xyz, self.axes.M)
        self.xy = (x2, y2)
        super().draw(renderer)

    def _annotate3D(ax, text, xyz, *args, **kwargs):
        '''
        Add anotation `text` to an `Axes3d` instance.
        '''
        annotation = Annotation3D(text, xyz, *args, **kwargs)
        ax.add_artist(annotation)

    setattr(Axes3D, 'annotate3D', _annotate3D)#}}}

def get_parser():
    parser = argparse.ArgumentParser(description='Plot 3d trajectory from result json file',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-f', '--file_path', default=default_path, type=str, help='Path of the result json file')
    return parser

def plot_3d_graph(file_path):
    result = open_json(file_path)
    map_width = result['env_args']['map_width']
    min_alt = result['env_args']['min_altitude']
    max_alt = result['env_args']['max_altitude']

    node_list = result['trajectory']
    x_list = [node['position'][0] for node in node_list]
    y_list = [node['position'][1] for node in node_list]
    z_list = [node['position'][2] for node in node_list]

    fig = plt.figure()
    ax = plt.axes(projection='3d')
#    ax.set_xlim(0,map_width)
#    ax.set_ylim(0,map_width)
#    ax.set_zlim(min_alt,max_alt)

    # Trajectory graph
    ax.plot3D(x_list, y_list, z_list, 'r')
    ax.plot3D(x_list, y_list, z_list, 'ro', markersize=3)
#    ax.scatter(x_list[1:-2], y_list[1:-2], z_list[1:-2], c=list(range(1,len(x_list)-3)), s=20, cmap=plt.cm.Reds)

    ax.plot3D(x_list[0], y_list[0], z_list[0], 'bo',  markersize=4)
    ax.annotate3D('Start', (x_list[0], y_list[0], z_list[0]), xytext=(3, 3), textcoords='offset points')
    ax.plot3D(x_list[-1], y_list[-1], z_list[-1], 'bo',  markersize=4)
    ax.annotate3D('End', (x_list[-1], y_list[-1], z_list[-1]), xytext=(3, 3), textcoords='offset points')

    # User distribution
    user_position_x_list = [user['position'][0] for user in node_list[0]['user_list']]
    user_position_y_list = [user['position'][1] for user in node_list[0]['user_list']]
    user_time_list = [user['time_start'] for user in node_list[0]['user_list']]
#    ax.scatter(user_position_x_list, user_position_y_list, [50]*len(user_position_x_list), c=user_time_list,
#            s=20, cmap=plt.cm.Greens,  markersize=4)
    ax.scatter(user_position_x_list, user_position_y_list, [50]*len(user_position_x_list), c=user_time_list,
            s=20, cmap=plt.cm.Greens)

    plt.show()

if __name__=='__main__':
    parser = get_parser()
    args = parser.parse_args()
    if not bool(args.file_path):
        parser.error('File path must be specified.')
        parser.error(f'Usage: {__file__} --file_path <file_path>')
        parser.error(f'Usage: {__file__} --file_path <file_path>')

#    plot_3d_graph(args.file_path)
    for idx in range(150): 
        for d in range(1,6,2):
            idx=134
            default_path = f'/storage/hslyu/bw2/ours-rate/datarate_10/user_20/depth_{d}/env_{idx:04d}-depth_{d}-ue_20.json'
            print(default_path)
            plot_3d_graph(default_path)
