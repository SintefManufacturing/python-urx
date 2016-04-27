"""
Python library to control an UR robot through its TCP/IP interface
DOC LINK
http://support.universal-robots.com/URRobot/RemoteAccess
"""


import math3d as m3d
import numpy as np

from urx.urrobot import URRobot

__author__ = "Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011-2016, Sintef Raufoss Manufacturing"
__license__ = "GPLv3"


class Robot(URRobot):

    """
    Generic Python interface to an industrial robot.
    Compared to the URRobot class, this class adds the possibilty to work directly with matrices
    and includes support for setting a reference coordinate system
    """

    def __init__(self, host, use_rt=False):
        URRobot.__init__(self, host, use_rt)
        self.csys = m3d.Transform()

    def _get_lin_dist(self, target):
        pose = URRobot.getl(self, wait=True)
        target = m3d.Transform(target)
        pose = m3d.Transform(pose)
        return pose.dist(target)

    def set_tcp(self, tcp):
        """
        set robot flange to tool tip transformation
        """
        if isinstance(tcp, m3d.Transform):
            tcp = tcp.pose_vector
        URRobot.set_tcp(self, tcp)

    def set_csys(self, transform):
        """
        Set reference coordinate system to use
        """
        self.csys = transform

    def set_orientation(self, orient, acc=0.01, vel=0.01, wait=True, threshold=None):
        """
        set tool orientation using a orientation matric from math3d
        or a orientation vector
        """
        if not isinstance(orient, m3d.Orientation):
            orient = m3d.Orientation(orient)
        trans = self.get_pose()
        trans.orient = orient
        self.set_pose(trans, acc, vel, wait=wait, threshold=threshold)

    def translate_tool(self, vect, acc=0.01, vel=0.01, wait=True, threshold=None):
        """
        move tool in tool coordinate, keeping orientation
        """
        t = m3d.Transform()
        if not isinstance(vect, m3d.Vector):
            vect = m3d.Vector(vect)
        t.pos += vect
        return self.add_pose_tool(t, acc, vel, wait=wait, threshold=threshold)

    def back(self, z=0.05, acc=0.01, vel=0.01):
        """
        move in z tool
        """
        self.translate_tool((0, 0, -z), acc=acc, vel=vel)

    def set_pos(self, vect, acc=0.01, vel=0.01, wait=True, threshold=None):
        """
        set tool to given pos, keeping constant orientation
        """
        if not isinstance(vect, m3d.Vector):
            vect = m3d.Vector(vect)
        trans = m3d.Transform(self.get_orientation(), m3d.Vector(vect))
        return self.set_pose(trans, acc, vel, wait=wait, threshold=threshold)

    def movec(self, pose_via, pose_to, acc=0.01, vel=0.01, wait=True, threshold=None):
        """
        Move Circular: Move to position (circular in tool-space)
        see UR documentation
        """
        pose_via = self.csys * m3d.Transform(pose_via)
        pose_to = self.csys * m3d.Transform(pose_to)
        pose = URRobot.movec(self, pose_via.pose_vector, pose_to.pose_vector, acc=acc, vel=vel, wait=wait, threshold=threshold)
        if pose is not None:
            return self.csys.inverse * m3d.Transform(pose)

    def set_pose(self, trans, acc=0.01, vel=0.01, wait=True, command="movel", threshold=None):
        """
        move tcp to point and orientation defined by a transformation
        UR robots have several move commands, by default movel is used but it can be changed
        using the command argument
        """
        self.logger.debug("Setting pose to %s", trans.pose_vector)
        t = self.csys * trans
        pose = URRobot.movex(self, command, t.pose_vector, acc=acc, vel=vel, wait=wait, threshold=threshold)
        if pose is not None:
            return self.csys.inverse * m3d.Transform(pose)

    def add_pose_base(self, trans, acc=0.01, vel=0.01, wait=True, command="movel", threshold=None):
        """
        Add transform expressed in base coordinate
        """
        pose = self.get_pose()
        return self.set_pose(trans * pose, acc, vel, wait=wait, command=command, threshold=threshold)

    def add_pose_tool(self, trans, acc=0.01, vel=0.01, wait=True, command="movel", threshold=None):
        """
        Add transform expressed in tool coordinate
        """
        pose = self.get_pose()
        return self.set_pose(pose * trans, acc, vel, wait=wait, command=command, threshold=threshold)

    def get_pose(self, wait=False, _log=True):
        """
        get current transform from base to to tcp
        """
        pose = URRobot.getl(self, wait, _log)
        trans = self.csys.inverse * m3d.Transform(pose)
        if _log:
            self.logger.debug("Returning pose to user: %s", trans.pose_vector)
        return trans

    def get_orientation(self, wait=False):
        """
        get tool orientation in base coordinate system
        """
        trans = self.get_pose(wait)
        return trans.orient

    def get_pos(self, wait=False):
        """
        get tool tip pos(x, y, z) in base coordinate system
        """
        trans = self.get_pose(wait)
        return trans.pos

    def speedx(self, command, velocities, acc, min_time):
        """
        send command to robot formated like speedl or speedj
        move at given velocities until minimum min_time seconds
        """
        v = self.csys.orient * m3d.Vector(velocities[:3])
        w = self.csys.orient * m3d.Vector(velocities[3:])
        URRobot.speedx(self, command, np.concatenate((v.array, w.array)), acc, min_time)

    def speedl_tool(self, velocities, acc, min_time):
        """
        move at given velocities in tool csys until minimum min_time seconds
        """
        pose = self.get_pose()
        v = pose.orient * m3d.Vector(velocities[:3])
        w = pose.orient * m3d.Vector(velocities[3:])
        URRobot.speedl(self, np.concatenate((v.array, w.array)), acc, min_time)

    def movex(self, command, pose, acc=0.01, vel=0.01, wait=True, relative=False, threshold=None):
        """
        Send a move command to the robot. since UR robotene have several methods this one
        sends whatever is defined in 'command' string
        """
        t = m3d.Transform(pose)
        if relative:
            return self.add_pose_base(t, acc, vel, wait=wait, command=command, threshold=threshold)
        else:
            return self.set_pose(t, acc, vel, wait=wait, command=command, threshold=threshold)

    def movexs(self, command, pose_list, acc=0.01, vel=0.01, radius=0.01, wait=True, threshold=None):
        """
        Concatenate several movex commands and applies a blending radius
        pose_list is a list of pose.
        This method is usefull since any new command from python
        to robot make the robot stop
        """
        new_poses = []
        for pose in pose_list:
            t = self.csys * m3d.Transform(pose)
            pose = t.pose_vector
            new_poses.append(pose)
        return URRobot.movexs(self, command, new_poses, acc, vel, radius, wait=wait, threshold=threshold)

    def movel_tool(self, pose, acc=0.01, vel=0.01, wait=True, threshold=None):
        """
        move linear to given pose in tool coordinate
        """
        return self.movex_tool("movel", pose, acc=acc, vel=vel, wait=wait, threshold=threshold)

    def movex_tool(self, command, pose, acc=0.01, vel=0.01, wait=True, threshold=None):
        t = m3d.Transform(pose)
        self.add_pose_tool(t, acc, vel, wait=wait, command=command, threshold=threshold)

    def getl(self, wait=False, _log=True):
        """
        return current transformation from tcp to current csys
        """
        t = self.get_pose(wait, _log)
        return t.pose_vector.tolist()

    def set_gravity(self, vector):
        if isinstance(vector, m3d.Vector):
            vector = vector.list
        return URRobot.set_gravity(self, vector)

    def new_csys_from_xpy(self):
        """
        Return new coordinate system from three points: X, Origin, Y
        based on math3d: Transform.new_from_xyp
        """
        print("A new coordinate system will be defined from the next three points")
        print("Firs point is X, second Origin, third Y")
        print("Set it as a new reference by calling myrobot.set_csys(new_csys)")
        input("Move to first point and click Enter")
        # we do not use get_pose so we avoid rounding values
        pose = URRobot.getl(self)
        p0 = m3d.Vector(pose[:3])
        input("Move to second point and click Enter")
        pose = URRobot.getl(self)
        p1 = m3d.Vector(pose[:3])
        input("Move to second point and click Enter")
        pose = URRobot.getl(self)
        p2 = m3d.Vector(pose[:3])
        return m3d.Transform.new_from_xyp(p1 - p0, p2 - p0, p0)

    @property
    def x(self):
        return self.get_pos().x

    @x.setter
    def x(self, val):
        p = self.get_pos()
        p.x = val
        self.set_pos(p)

    @property
    def y(self):
        return self.get_pos().y

    @y.setter
    def y(self, val):
        p = self.get_pos()
        p.y = val
        self.set_pos(p)

    @property
    def z(self):
        return self.get_pos().z

    @z.setter
    def z(self, val):
        p = self.get_pos()
        p.z = val
        self.set_pos(p)

    @property
    def rx(self):
        return 0

    @rx.setter
    def rx(self, val):
        p = self.get_pose()
        p.orient.rotate_xb(val)
        self.set_pose(p)

    @property
    def ry(self):
        return 0

    @ry.setter
    def ry(self, val):
        p = self.get_pose()
        p.orient.rotate_yb(val)
        self.set_pose(p)

    @property
    def rz(self):
        return 0

    @rz.setter
    def rz(self, val):
        p = self.get_pose()
        p.orient.rotate_zb(val)
        self.set_pose(p)

    @property
    def x_t(self):
        return 0

    @x_t.setter
    def x_t(self, val):
        t = m3d.Transform()
        t.pos.x += val
        self.add_pose_tool(t)

    @property
    def y_t(self):
        return 0

    @y_t.setter
    def y_t(self, val):
        t = m3d.Transform()
        t.pos.y += val
        self.add_pose_tool(t)

    @property
    def z_t(self):
        return 0

    @z_t.setter
    def z_t(self, val):
        t = m3d.Transform()
        t.pos.z += val
        self.add_pose_tool(t)

    @property
    def rx_t(self):
        return 0

    @rx_t.setter
    def rx_t(self, val):
        t = m3d.Transform()
        t.orient.rotate_xb(val)
        self.add_pose_tool(t)

    @property
    def ry_t(self):
        return 0

    @ry_t.setter
    def ry_t(self, val):
        t = m3d.Transform()
        t.orient.rotate_yb(val)
        self.add_pose_tool(t)

    @property
    def rz_t(self):
        return 0

    @rz_t.setter
    def rz_t(self, val):
        t = m3d.Transform()
        t.orient.rotate_zb(val)
        self.add_pose_tool(t)








