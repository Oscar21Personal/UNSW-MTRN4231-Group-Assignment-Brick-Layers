import math
import sys
from geometry_msgs.msg import TransformStamped
from interfaces.msg import PartSize
from interfaces.srv import GetPartSize
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster

import trimesh
import os


class BlockFramePublisher(Node):
    """
    Static Broadcaster of of the inventory locations
    """

    def __init__(self):
        super().__init__('static_target_points_broadcaster')


        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.srv = self.create_service(GetPartSize, 'get_part_size', self.make_transforms)

        # Publish static transforms once at startup
        # self.make_transforms()

    def make_transforms(self, request, response):
        
        self.make_mesh(request.file_path)
        
        ts = []

        block_id = 0
        for z in range(10,int(self.max_point[2])+10,20):
            for y in range(10,int(self.max_point[1])+10,20):
                for x in range(10,int(self.max_point[0])+10,20):
                    if self.block_in_model([x,y,z]):
                        t = TransformStamped()

                        t.header.stamp = self.get_clock().now().to_msg()
                        t.header.frame_id = "board_frame"
                        t.child_frame_id = "target_block" + str(block_id)
                        
                        t.transform.translation.x = float(x)/1000 + 0.1
                        t.transform.translation.y = float(y)/1000 + 0.1
                        t.transform.translation.z = float(z)/1000

                        block_id +=1

                        ts.append(t)

        self.tf_static_broadcaster.sendTransform(ts)
        print(ts[0])
        print(ts[1])
        print('publishing tf')

        response.size_x = len(range(10,int(self.max_point[0])+10,20))
        response.size_y = len(range(10,int(self.max_point[1])+10,20))
        response.size_z = len(range(10,int(self.max_point[2])+10,20))

        return response

    def make_mesh(self, stl_path):

        self.mesh = trimesh.load(stl_path)
        x, y, z = map(list, zip(*self.mesh.vertices))

        minx = min(x)
        miny = min(y)
        minz = min(z)
        min_point =  [minx,miny,minz]

        self.mesh.vertices -= min_point
        x, y, z = map(list, zip(*self.mesh.vertices))

        self.min_point = [0.0,0.0,0.0]

        maxx = max(x)
        maxy = max(y)
        maxz = max(z)
        self.max_point =  [maxx,maxy,maxz]

    def block_in_model(self, block_loc):

        ray_direction = np.array([[1, 0, 0]])
        fill_count = 0
        x_points = range(block_loc[0]-10,block_loc[0]+10,4)
        y_points = range(block_loc[1]-10,block_loc[1]+10,4)
        z_points = range(block_loc[2]-10,block_loc[2]+10,4)

        total_count  = len(x_points) * len(y_points) * len(z_points)
        for fill_x in x_points:
            for fill_y in y_points:
                for fill_z in z_points:
                    fill_origin = np.array([[fill_x, fill_y, fill_z]])
                    intersect_locations, index_ray, index_tri = self.mesh.ray.intersects_location(ray_origins=fill_origin, ray_directions=ray_direction)
                    if len(intersect_locations) % 2 == 1:
                        fill_count += 1
        # print(f"fillcount: {fill_count}")
        if fill_count/total_count > .3:
            return True
        else:
            return False



def path_script():
    _path = os.path.dirname(os.path.abspath(__file__))
    _path = _path.replace('\\', '/')
    return _path

def main():

    # stl_path = '/home/josh/4231/group-assignment-brick-layers/src/STL_parser/stl_files/mvp.stl' #+ ('../home/2_cubes.STL')
    # mesh = trimesh.load(stl_path)

    logger = rclpy.logging.get_logger('logger')

    # pass parameters and initialize node
    rclpy.init()
    block_loc_node = BlockFramePublisher()

    try:
        rclpy.spin(block_loc_node)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()