#!/usr/bin/env python3
"""Derive/check the fixed TCP from both finger STL bounds; never opens hardware."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / 'src/A5'
URDF = DESCRIPTION / 'urdf/A5.urdf'
FINGERS = ('gripper_finger_joint1', 'gripper_finger_joint2')
BEGIN = '  <!-- BEGIN STL-DERIVED TCP: tools/derive_a5_tcp.py -->'
END = '  <!-- END STL-DERIVED TCP -->'


def read_stl(path):
    """Read binary or ASCII STL triangles without weighting by tessellation."""
    blob = Path(path).read_bytes()
    count = struct.unpack_from('<I', blob, 80)[0] if len(blob) >= 84 else 0
    if count and len(blob) == 84 + 50 * count:
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)),
                          ('attribute', '<u2')])
        triangles = np.frombuffer(blob, dtype=dtype, offset=84)['vertices'].astype(float)
    else:
        vertices = [list(map(float, line.split()[1:])) for line in blob.decode('ascii').splitlines()
                    if line.strip().startswith('vertex ')]
        if not vertices or len(vertices) % 3:
            raise ValueError(f'Invalid or empty STL: {path}')
        triangles = np.asarray(vertices, dtype=float).reshape(-1, 3, 3)
    if not np.isfinite(triangles).all():
        raise ValueError(f'Non-finite STL coordinates: {path}')
    return triangles


def vector(element, attribute, default):
    return np.fromstring(element.get(attribute, default) if element is not None else default, sep=' ')


def rotation(rpy):
    r, p, y = rpy
    cr, cp, cy = np.cos([r, p, y]); sr, sp, sy = np.sin([r, p, y])
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def link_triangles(robot, description, name):
    link = robot.find(f"link[@name='{name}']")
    triangles = []
    sources = []
    for visual in link.findall('visual'):
        mesh = visual.find('geometry/mesh')
        if mesh is None:
            raise ValueError(f'{name}: expected STL visual geometry')
        uri = mesh.get('filename')
        prefix = 'package://a5_description/'
        if not uri.startswith(prefix):
            raise ValueError(f'Unexpected mesh URI: {uri}')
        path = description / uri.removeprefix(prefix)
        tri = read_stl(path) * vector(mesh, 'scale', '1 1 1')
        origin = visual.find('origin')
        tri = tri @ rotation(vector(origin, 'rpy', '0 0 0')).T + vector(origin, 'xyz', '0 0 0')
        triangles.append(tri)
        sources.append({'uri': uri, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                        'triangles': len(tri)})
    if not triangles:
        raise ValueError(f'{name}: no visual mesh')
    return np.concatenate(triangles), sources


def finger_geometry(robot, description, q_finger1):
    triangles = []
    details = []
    for name in FINGERS:
        joint = robot.find(f"joint[@name='{name}']")
        if joint.get('type') != 'prismatic' or joint.find('parent').get('link') != 'link6':
            raise ValueError(f'{name}: expected prismatic child of link6')
        q = q_finger1
        if name == FINGERS[1]:
            mimic = joint.find('mimic')
            if mimic is None or mimic.get('joint') != FINGERS[0]:
                raise ValueError('Second finger must mimic the first')
            q = q * float(mimic.get('multiplier', '1')) + float(mimic.get('offset', '0'))
        child = joint.find('child').get('link')
        tri, sources = link_triangles(robot, description, child)
        origin = joint.find('origin')
        axis = vector(joint.find('axis'), 'xyz', '1 0 0')
        if not np.isclose(np.linalg.norm(axis), 1):
            raise ValueError(f'{name}: joint axis is not normalized')
        tri = ((tri + q * axis) @ rotation(vector(origin, 'rpy', '0 0 0')).T
               + vector(origin, 'xyz', '0 0 0'))
        bounds = np.array([tri.min(axis=(0, 1)), tri.max(axis=(0, 1))])
        triangles.append(tri)
        details.append({'joint': name, 'position_m': q, 'bounds_in_link6_m': bounds.tolist(),
                        'sources': sources})
    all_triangles = np.concatenate(triangles)
    bounds = np.array([all_triangles.min(axis=(0, 1)), all_triangles.max(axis=(0, 1))])
    return bounds.mean(axis=0), bounds, details, triangles


def derive(urdf=URDF, description=DESCRIPTION):
    robot = ET.parse(urdf).getroot()
    samples = []
    for q in (0.0, -0.015, -0.03):
        center, bounds, fingers, _ = finger_geometry(robot, description, q)
        samples.append({'finger1_position_m': q, 'center_m': center.tolist(),
                        'combined_bounds_m': bounds.tolist(), 'fingers': fingers})
    centers = np.array([s['center_m'] for s in samples])
    drift = float(np.max(np.linalg.norm(centers - centers[0], axis=1)))
    if drift > 1e-7:
        raise ValueError(f'Finger center moves with opening ({drift} m); cannot use a fixed TCP')
    return {'method': 'paired_finger_visual_aabb_center_in_link6',
            'parent_frame': 'link6', 'child_frame': 'tool_tcp',
            'xyz_m': centers[0].tolist(), 'rpy_rad': [0.0, 0.0, 0.0],
            'mesh_coordinates': 'URDF mesh scale applied; model length unit is metre',
            'opening_center_max_drift_m': drift, 'samples': samples,
            'hardware_accessed': False, 'physical_contact_point_calibrated': False}


def check_tcp(robot, result):
    joint = robot.find("joint[@name='tool_tcp_joint']")
    if joint is None or joint.get('type') != 'fixed':
        raise ValueError('Missing fixed tool_tcp_joint; run derive_a5_tcp.py --write')
    if joint.find('parent').get('link') != 'link6' or joint.find('child').get('link') != 'tool_tcp':
        raise ValueError('TCP must be a fixed child of link6')
    if not np.allclose(vector(joint.find('origin'), 'xyz', '0 0 0'), result['xyz_m'], atol=1e-10, rtol=0):
        raise ValueError('URDF TCP differs from STL-derived center')
    if not np.allclose(vector(joint.find('origin'), 'rpy', '0 0 0'), 0, atol=1e-12):
        raise ValueError('TCP axes must inherit link6 axes')
    link = robot.find("link[@name='tool_tcp']")
    if link is None or len(link):
        raise ValueError('tool_tcp must be an empty, massless frame')


def write_tcp(urdf, result):
    source = urdf.read_text()
    xyz = ' '.join(f'{v:.12g}' for v in result['xyz_m'])
    block = (f'{BEGIN}\n  <link name="tool_tcp" />\n'
             f'  <joint name="tool_tcp_joint" type="fixed">\n'
             f'    <parent link="link6" />\n    <child link="tool_tcp" />\n'
             f'    <origin xyz="{xyz}" rpy="0 0 0" />\n  </joint>\n{END}')
    if BEGIN in source:
        source, count = re.subn(re.escape(BEGIN) + r'.*?' + re.escape(END), block, source, flags=re.S)
        if count != 1:
            raise ValueError('Expected one generated TCP block')
    else:
        if 'name="tool_tcp"' in source or 'name="tool_tcp_joint"' in source:
            raise ValueError('Refusing to replace an unmanaged TCP definition')
        source = source.replace('</robot>', block + '\n</robot>')
    urdf.write_text(source.rstrip() + '\n')


def plot_geometry(result, output):
    # Optional, not needed by build/verify or TCP derivation.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    robot = ET.parse(URDF).getroot()
    _, _, _, fingers = finger_geometry(robot, DESCRIPTION, -0.015)
    wrist, _ = link_triangles(robot, DESCRIPTION, 'link6')
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, dims in zip(axes, [(0, 1), (0, 2)]):
        for tri, color in zip([wrist] + fingers, ['#999999', '#297ab3', '#f29339']):
            ax.add_collection(PolyCollection(tri[:, :, dims] * 1000, facecolor=color,
                                             edgecolor='none', alpha=0.18, rasterized=True))
        tcp = np.array(result['xyz_m']) * 1000
        ax.scatter(tcp[dims[0]], tcp[dims[1]], c='red', s=50, zorder=4, label='STL-derived TCP')
        ax.autoscale(); ax.set_aspect('equal'); ax.grid(alpha=0.2)
        ax.set_xlabel('XYZ'[dims[0]] + ' / mm'); ax.set_ylabel('XYZ'[dims[1]] + ' / mm')
    axes[0].legend(); fig.suptitle('A5 meshes in link6 frame; finger1 = -15 mm')
    fig.tight_layout(); output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write', action='store_true', help='Update only the generated TCP block in source URDF')
    parser.add_argument('--plot', type=Path, help='Optional geometry preview (requires matplotlib)')
    args = parser.parse_args()
    result = derive()
    if args.write:
        write_tcp(URDF, result)
    check_tcp(ET.parse(URDF).getroot(), result)
    out = ROOT / 'validation/tcp_geometry.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(result, indent=2) + '\n')
    if args.plot:
        plot_geometry(result, args.plot)
    print(f"TCP link6 -> tool_tcp xyz_m={result['xyz_m']}, rpy_rad={result['rpy_rad']}")
    print(f"Opening invariance: {result['opening_center_max_drift_m']:.3g} m; {out}")


if __name__ == '__main__':
    main()
