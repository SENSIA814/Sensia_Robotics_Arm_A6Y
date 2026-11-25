"""RViz mesh previews using Pinocchio FK, without MoveIt display callbacks."""
import time
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

from motion_model import Kinematics


class ModelPreview:
    def __init__(self, node, model):
        from visualization_msgs.msg import MarkerArray
        self.publisher = node.create_publisher(MarkerArray, 'model_preview', 1)
        self.kin = Kinematics(model)
        self.visuals = []
        for link in ET.fromstring(model['description']).findall('link'):
            for visual in link.findall('visual'):
                origin = visual.find('origin')
                xyz = np.fromstring(origin.get('xyz', '0 0 0') if origin is not None else '0 0 0', sep=' ')
                rpy = np.fromstring(origin.get('rpy', '0 0 0') if origin is not None else '0 0 0', sep=' ')
                local = pin.SE3(pin.rpy.rpyToMatrix(rpy), xyz)
                self.visuals.append((self.kin.model.getFrameId(link.get('name')), local, visual.find('geometry')))
        self.last = 0.
        self.animation_start = time.monotonic()
        self.path_plan = None
        self.path_points = []

    def robot(self, q, namespace, color):
        from visualization_msgs.msg import Marker
        from pose_utils import pose_message
        self.kin.fk(q)  # Updates all frame placements, including mimic fingers.
        result = []
        for i, (frame, origin, geometry) in enumerate(self.visuals):
            m = Marker()
            m.header.frame_id, m.ns, m.id = 'base_link', namespace, i
            m.pose = pose_message((self.kin.data.oMf[frame] * origin).homogeneous)
            m.color.r, m.color.g, m.color.b, m.color.a = color
            mesh, box, cylinder, sphere = (geometry.find(t) for t in ('mesh', 'box', 'cylinder', 'sphere'))
            if mesh is not None:
                m.type, m.mesh_resource = Marker.MESH_RESOURCE, mesh.get('filename')
                m.scale.x, m.scale.y, m.scale.z = map(float, mesh.get('scale', '1 1 1').split())
            elif box is not None:
                m.type = Marker.CUBE
                m.scale.x, m.scale.y, m.scale.z = map(float, box.get('size').split())
            elif cylinder is not None:
                m.type = Marker.CYLINDER
                m.scale.x = m.scale.y = 2 * float(cylinder.get('radius'))
                m.scale.z = float(cylinder.get('length'))
            elif sphere is not None:
                m.type = Marker.SPHERE
                m.scale.x = m.scale.y = m.scale.z = 2 * float(sphere.get('radius'))
            else:
                continue
            result.append(m)
        return result

    def publish(self, goal, target, valid, plan, source):
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker, MarkerArray
        now = time.monotonic()
        if now - self.last < .1:
            return
        self.last = now
        markers = []
        if goal is not None:
            markers.extend(self.robot(goal, 'goal', (.15, .95, .4, .48)))
        if target is not None:
            m = Marker()
            m.header.frame_id, m.ns, m.id = 'base_link', 'reachability', 0
            m.type = Marker.SPHERE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, target[:3, 3])
            m.pose.orientation.w = 1.
            m.scale.x = m.scale.y = m.scale.z = .068
            m.color.r, m.color.g, m.color.b, m.color.a = (.1, .95, .3, .65) if valid else (1., .2, .1, .85)
            markers.append(m)
        if plan is not self.path_plan:
            self.path_plan, self.animation_start = plan, now
            self.path_points = []
            if plan is not None:
                for elapsed in np.linspace(0, plan.duration, min(200, int(plan.duration / .05) + 2)):
                    q = source.copy()
                    q[plan.indices], _ = plan.sample(elapsed)
                    xyz = self.kin.fk(q)[:3, 3]
                    self.path_points.append(Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])))
        if plan is not None:
            q = source.copy()
            q[plan.indices], _ = plan.sample((now - self.animation_start) % (plan.duration + .7))
            markers.extend(self.robot(q, 'trajectory', (.15, .55, 1., .4)))
            m = Marker()
            m.header.frame_id, m.ns, m.id = 'base_link', 'path', 0
            m.type, m.scale.x, m.pose.orientation.w = Marker.LINE_STRIP, .003, 1.
            m.color.b, m.color.g, m.color.a = 1., .6, .9
            m.points = self.path_points
            markers.append(m)
        else:
            for namespace, count in [('trajectory', len(self.visuals)), ('path', 1)]:
                for i in range(count):
                    m = Marker()
                    m.ns, m.id, m.action = namespace, i, Marker.DELETE
                    markers.append(m)
        self.publisher.publish(MarkerArray(markers=markers))
