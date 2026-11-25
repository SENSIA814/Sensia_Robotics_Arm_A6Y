"""URDF mesh distances and conservative swept-path margins (no world perception)."""
import itertools
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pinocchio as pin
import coal
from scipy.spatial import ConvexHull
from .a5_kinematics import JOINTS


class DistanceCertificates:
    """Bounded point-distance history; every reused bound includes arc travel."""
    def __init__(self, weights, capacity=128):
        self.weights = np.asarray(weights)
        self.positions = np.zeros((capacity, 7))
        self.distances = np.zeros(capacity)
        self.count = self.cursor = 0

    def remember(self, position, distance):
        # Store only queried distances, never a weakened/reused lower bound.
        self.positions[self.cursor] = position
        self.distances[self.cursor] = distance
        self.count = min(self.count+1, len(self.distances))
        self.cursor = (self.cursor+1) % len(self.distances)

    def lower_bound(self, start, end=None):
        if not self.count: return -float('inf')
        travel = np.abs(self.positions[:self.count]-start)
        if end is not None:
            travel = np.maximum(travel, np.abs(self.positions[:self.count]-end))
        bounds = self.distances[:self.count]-np.sum(travel*self.weights, axis=1)
        return float(np.max(bounds))


class GeometryGuard:
    def __init__(self, kin, urdf, config):
        self.kin, self.cfg = kin, config
        root = ET.parse(urdf).getroot()
        adjacent = {frozenset((j.find('parent').get('link'), j.find('child').get('link')))
                    for j in root.findall('joint')}
        # The fingers are allowed to contact only each other at closed gap.
        adjacent.add(frozenset(('gripper_finger_link1', 'gripper_finger_link2')))
        for mesh in root.findall('.//mesh'):
            filename = mesh.get('filename', '')
            if filename.startswith('package://a5_description/'):
                mesh.set('filename', str(Path(urdf).resolve().parents[1] / filename.split('package://a5_description/')[1]))
        self.geom = pin.buildGeomFromUrdfString(kin.model, ET.tostring(root, encoding='unicode'),
                                               pin.GeometryType.COLLISION)
        if len(self.geom.geometryObjects) != 9:
            raise ValueError('expected nine A5 collision meshes')
        self.geom.addAllCollisionPairs()
        for first, second in [(p.first, p.second) for p in self.geom.collisionPairs]:
            a, b = (self.geom.geometryObjects[i] for i in (first, second))
            names = frozenset((kin.model.frames[a.parentFrame].name, kin.model.frames[b.parentFrame].name))
            if a.parentFrame == b.parentFrame or names in adjacent:
                self.geom.removeCollisionPair(pin.CollisionPair(first, second))
        self._configure_zero_tolerance(config.get('zero_pose_collision_tolerance', {}))
        parent = {j.find('child').get('link'): (j.find('parent').get('link'), j.get('name'))
                  for j in root.findall('joint')}
        def ancestors(link):
            result = set()
            while link in parent:
                link, joint = parent[link]
                if joint == 'gripper_finger_joint2': joint = 'gripper_finger_joint1'
                if joint in JOINTS: result.add(JOINTS.index(joint))
            return result
        self.relative_axes = []
        for pair in self.geom.collisionPairs:
            a, b = (kin.model.frames[self.geom.geometryObjects[i].parentFrame].name
                    for i in (pair.first, pair.second))
            self.relative_axes.append(ancestors(a) ^ ancestors(b))
        # Convex envelopes give lower distance bounds. Exact mesh distances are
        # needed only when that conservative envelope overlaps the margin.
        self.hulls = pin.GeometryModel(self.geom)
        for obj in self.hulls.geometryObjects:
            vertices = np.asarray(obj.geometry.vertices())
            hull = ConvexHull(vertices)
            indices = {j: i for i, j in enumerate(hull.vertices)}
            points, faces = coal.StdVec_Vec3s(), coal.StdVec_Triangle()
            for j in hull.vertices: points.append(vertices[j])
            for triangle, equation in zip(hull.simplices, hull.equations):
                a, b, c = triangle
                if np.dot(np.cross(vertices[b]-vertices[a], vertices[c]-vertices[a]), equation[:3]) < 0:
                    b, c = c, b
                faces.append(coal.Triangle(indices[a], indices[b], indices[c]))
            obj.geometry = coal.Convex(points, faces)
        self.hull_data = pin.GeometryData(self.hulls)
        self.data = pin.GeometryData(self.geom)
        self.model_data = kin.model.createData()
        self.corners = []
        for obj in self.geom.geometryObjects:
            obj.geometry.computeLocalAABB()
            box = obj.geometry.aabb_local
            self.corners.append(np.array(list(itertools.product(*zip(box.min_, box.max_)))))
        self.last_distance = float('inf')
        for key in ('mesh_clearance_m', 'path_joint_step_rad', 'reach_bound_m'):
            if not np.isfinite(config[key]) or config[key] <= 0:
                raise ValueError('invalid geometry setting: '+key)
        # Keep certificates across the actual-to-reference corridor. One last
        # point thrashes between its ends when physical motors lag commands.
        self.distance_cache = [DistanceCertificates([
            (2*config['reach_bound_m'] if i < 6 else 2.) if i in axes else 0.
            for i in range(7)]) for axes in self.relative_axes]

    def _configure_zero_tolerance(self, cfg):
        """Only operator-confirmed folded-pose model discrepancies are eligible."""
        if not isinstance(cfg, dict) or type(cfg.get('enabled', False)) is not bool:
            raise ValueError('invalid zero-pose tolerance configuration')
        self.zero_tolerance_enabled = cfg.get('enabled', False)
        self.zero_band = cfg.get('zero_band_rad', float(np.deg2rad(2.)))
        self.anchor_band = cfg.get('anchor_band_rad', float(np.deg2rad(2.)))
        for value in (self.zero_band, self.anchor_band):
            if not isinstance(value, (int, float)) or not np.isfinite(value) or not 0 < value <= np.deg2rad(2.):
                raise ValueError('zero-pose tolerance must be positive and at most 2 degrees')
        allowed = {frozenset(p) for p in (('link1', 'link4'), ('link1', 'link5'), ('link2', 'link4'))}
        requested = cfg.get('pairs', [])
        if not isinstance(requested, list) or any(not isinstance(p, list) or len(p) != 2
                or any(not isinstance(n, str) for n in p) or frozenset(p) not in allowed for p in requested):
            raise ValueError('unapproved zero-pose collision pair')
        selected = {frozenset(p) for p in requested}
        self.zero_pairs = {}
        for index, pair in enumerate(self.geom.collisionPairs):
            names = [self.kin.model.frames[self.geom.geometryObjects[j].parentFrame].name
                     for j in (pair.first, pair.second)]
            if frozenset(names) in selected: self.zero_pairs[index] = names
        self.zero_anchor = None
        self.zero_epoch = None
        self.zero_seen_enabled = False
        self.last_tolerated_pairs = []

    def observe_controller(self, epoch, state, session_origin):
        """Pin the first PREPARE origin, never a later grip or DISARM origin."""
        if not self.zero_tolerance_enabled: return
        if epoch != self.zero_epoch:
            self.zero_epoch = epoch
            self.zero_anchor = None
            self.zero_seen_enabled = False
        if state in (1, 2, 3, 4, 5) and not self.zero_seen_enabled:
            self.zero_seen_enabled = True
            origin = np.asarray(session_origin, dtype=float)
            if origin.shape == (6,) and np.isfinite(origin).all() and np.all(np.abs(origin[1:3]) <= self.zero_band):
                self.zero_anchor = origin.copy()

    def _tolerated_pairs(self, start, end):
        if not self.zero_tolerance_enabled or any(np.any(np.abs(q[1:3]) > self.zero_band) for q in (start, end)):
            return set()
        if self.zero_anchor is None:
            # Before PREPARE, permit static readback/handshake only. Moving paths
            # require an origin from the real controller's prepared session.
            if self.zero_seen_enabled or not np.array_equal(start, end): return set()
        elif any(np.any(np.abs(q[:6]-self.zero_anchor) > self.anchor_band) for q in (start, end)):
            return set()
        return set(self.zero_pairs)

    def check(self, start, end):
        """Conservatively add an arc-length bound between sampled poses.

        reach_bound_m must bound every mesh point's distance from each joint axis.
        Twice that bound covers both members of a collision pair; commissioning
        must include model error and physical stopping distance in clearance.
        """
        start, end = np.asarray(start), np.asarray(end)
        self.last_tolerated_pairs = []
        if not np.isfinite([start, end]).all(): return False, 'nonfinite_path'
        # The zero band and anchored box are convex: checking both endpoints
        # keeps the complete segment inside. No measured coordinate is clamped.
        tolerated = self._tolerated_pairs(start, end)
        self.last_tolerated_pairs = [self.zero_pairs[i] for i in sorted(tolerated)]
        n = max(1, int(np.ceil(np.max(np.abs(end[:6]-start[:6]))/self.cfg['path_joint_step_rad'])))
        if n > 128: return False, 'path_too_long'
        swept = (2*self.cfg['reach_bound_m']*np.sum(np.abs(end[:6]-start[:6])) +
                 2*abs(end[6]-start[6])) / n / 2
        floor_margin = self.cfg['mesh_clearance_m'] + swept
        self.last_distance = float('inf')
        uncertain = set()
        for index, axes in enumerate(self.relative_axes):
            if index in tolerated: continue
            if not self.distance_cache[index].count:
                uncertain.add(index)
                continue
            guaranteed = self.distance_cache[index].lower_bound(start, end)
            self.last_distance = min(self.last_distance, guaranteed)
            if guaranteed < self.cfg['mesh_clearance_m']: uncertain.add(index)
        # A previous distance plus a conservative arc-length bound certifies
        # the entire new segment, without repeating expensive mesh queries.
        if not uncertain and self.cfg.get('floor_z_m') is None: return True, 'clear'
        for t in np.linspace(0, 1, n+1):
            sample = start+t*(end-start)
            q = self.kin.configuration(sample)
            # Cached pairs already have a conservative certificate for this
            # entire segment. Update placements once, query only uncertain pairs.
            pin.forwardKinematics(self.kin.model, self.model_data, q)
            pin.updateGeometryPlacements(self.kin.model, self.model_data, self.hulls, self.hull_data)
            pin.updateGeometryPlacements(self.kin.model, self.model_data, self.geom, self.data)
            for index, pair in enumerate(self.geom.collisionPairs):
                if index not in uncertain: continue
                axes = self.relative_axes[index]
                motion = sum(abs(end[i]-start[i]) for i in axes if i < 6)
                # Shared upstream joints rotate both links rigidly and cannot
                # change their separation. Only relative axes need a margin.
                margin = self.cfg['mesh_clearance_m'] + self.cfg['reach_bound_m']*motion/n
                if 6 in axes: margin += abs(end[6]-start[6])/n
                # The pair may be uncertain for the whole segment while a
                # previous exact distance certifies this individual sample.
                # Use the same arc-length bound and swept margin as above;
                # never store this weakened bound as a new exact distance.
                guaranteed = self.distance_cache[index].lower_bound(sample)
                if guaranteed >= margin:
                    self.last_distance = min(self.last_distance, guaranteed)
                    continue
                distance = pin.computeDistance(self.hulls, self.hull_data, index).min_distance
                if distance < margin:
                    distance = pin.computeDistance(self.geom, self.data, index).min_distance
                self.last_distance = min(self.last_distance, distance)
                if not np.isfinite(distance) or distance < margin:
                    a, b = (self.geom.geometryObjects[i].name for i in (pair.first, pair.second))
                    return False, 'mesh_clearance:'+a+':'+b
                self.distance_cache[index].remember(sample, distance)
            floor = self.cfg.get('floor_z_m')
            if floor is not None:
                for index, obj in enumerate(self.geom.geometryObjects):
                    if obj.parentJoint == 0: continue  # fixed base is mounted to the support
                    pose = self.data.oMg[index]
                    corners = self.corners[index] @ pose.rotation.T + pose.translation
                    if np.min(corners[:, 2])-floor < floor_margin:
                        return False, 'floor_clearance:'+obj.name
        return True, 'clear'
