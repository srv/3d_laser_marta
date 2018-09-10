import numpy as np
import open3d
import time

class ReconstructionViewer(object):
    def __init__(self):
        self.model = open3d.PointCloud()
        self.vis = open3d.Visualizer()
        self.vis.create_window(window_name= 'Laser 3D Pointcloud', width= 800L, height= 600L, left= 50L, right= 50L)
        self.ctr = self.vis.get_view_control()
        self.ctr.rotate(10,0)
        self.ctr.translate(-1,-1,1)

    def append(self, xyz_cloud, pose):
        pcd = open3d.PointCloud()
        pcd.points = open3d.Vector3dVector(xyz_cloud)
        # print '======================================='
        # print np.asarray(pcd.points)
        # print '--------------------------------------'
        pcd.transform(pose)
        # print np.asarray(pcd.points)
        # print '--------------------------------------'
        self.model = self.model + pcd

    def drawnow(self):
        # Actualitzar el visor
        self.vis.add_geometry(self.model)
        self.vis.update_geometry()
        self.vis.reset_view_point(False)
        self.vis.poll_events()
        self.vis.update_renderer()
        time.sleep(0.001)

    def run(self):
        self.vis.run()

    def save(self):
        open3d.write_point_cloud("reconstruction.pcd", self.model)

    def savePCD(self, name):
        open3d.write_point_cloud(name + '.pcd', self.model)



