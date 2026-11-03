#!/usr/bin/env python3
"""Write the Gazebo world and marker textures from the YAML config.

Called from demo.launch at parse time, so the files exist before gzserver starts.
Prints the world path on stdout (roslaunch stores it as a parameter).
"""
import argparse
import os

import cv2
import yaml

from rail_align_demo.geometry import make_aruco_detector, render_marker

OVERVIEW_POSE = '-11.0 -8.0 8.0 0 0.57 0.72'  # high and behind: whole path and both marker faces
QUIET_CELLS = 1
CELL_PIXELS = 80


def write_marker_assets(cfg, out_dir):
    """Create one texture per marker with a white quiet zone, and an OGRE material for each."""
    marker = cfg['marker']
    dictionary, _ = make_aruco_detector(marker['dictionary'])
    cells = _marker_cells(dictionary)
    textures = os.path.join(out_dir, 'materials', 'textures')
    scripts = os.path.join(out_dir, 'materials', 'scripts')
    os.makedirs(textures, exist_ok=True)
    os.makedirs(scripts, exist_ok=True)

    material = []
    for marker_id in (int(marker['left_id']), int(marker['right_id'])):
        inner = render_marker(dictionary, marker_id, cells * CELL_PIXELS)
        pad = QUIET_CELLS * CELL_PIXELS
        image = cv2.copyMakeBorder(inner, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
        cv2.imwrite(os.path.join(textures, 'marker_%d.png' % marker_id), image)
        material.append("""material RailAlign/Marker_%d
{
  technique
  {
    pass
    {
      ambient 1 1 1 1
      diffuse 1 1 1 1
      specular 0 0 0 1
      texture_unit
      {
        texture marker_%d.png
      }
    }
  }
}
""" % (marker_id, marker_id))
    with open(os.path.join(scripts, 'markers.material'), 'w') as handle:
        handle.write('\n'.join(material))
    return cells


def _marker_cells(dictionary):
    """Grid size of the black square, including its one-cell black border."""
    return int(dictionary.markerSize) + 2


def _box_visual(name, center, size, material):
    return """      <visual name="%s">
        <pose>%.4f %.4f %.4f 0 0 0</pose>
        <geometry><box><size>%.4f %.4f %.4f</size></box></geometry>
        <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>%s</name></script></material>
      </visual>
""" % ((name,) + tuple(center) + tuple(size) + (material,))


def rail_model(cfg):
    """Two long flat lines from the entry line into +X. Visual only, so nothing catches the wheels."""
    rail = cfg['rail']
    gauge = float(rail['gauge_m'])
    width = float(rail['width_m'])
    length = float(rail['length_m'])
    visuals = [
        _box_visual('rail_left', (length / 2, gauge / 2, 0.002), (length, width, 0.004), 'Gazebo/DarkGrey'),
        _box_visual('rail_right', (length / 2, -gauge / 2, 0.002), (length, width, 0.004), 'Gazebo/DarkGrey'),
    ]
    return """    <model name="rail_entry">
      <static>true</static>
      <link name="link">
%s      </link>
    </model>
""" % ''.join(visuals)


def marker_model(cfg, name, y, marker_id, cells, out_dir):
    marker = cfg['marker']
    size = float(marker['size_m'])
    plate = size * (cells + 2 * QUIET_CELLS) / float(cells)
    x = float(marker['x_m'])
    z = float(marker['center_height_m'])
    thickness = 0.006
    # The marker plane is the plate's -X face, so that face sits exactly at x_m.
    uri = 'file://' + os.path.abspath(out_dir)
    post_height = z - plate / 2.0
    return """    <model name="%s">
      <static>true</static>
      <pose>%.4f %.4f %.4f 0 0 0</pose>
      <link name="link">
        <visual name="plate">
          <pose>%.4f 0 0 0 0 0</pose>
          <geometry><box><size>%.4f %.4f %.4f</size></box></geometry>
          <material>
            <script>
              <uri>%s/materials/scripts</uri>
              <uri>%s/materials/textures</uri>
              <name>RailAlign/Marker_%d</name>
            </script>
          </material>
        </visual>
        <visual name="post">
          <pose>0.02 0 %.4f 0 0 0</pose>
          <geometry><box><size>0.03 0.03 %.4f</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>
        </visual>
      </link>
    </model>
""" % (name, x, y, z, thickness / 2.0, thickness, plate, plate, uri, uri, marker_id,
       -(plate + post_height) / 2.0, post_height)


def overview_camera():
    """Fixed camera covering the whole run, used only for recording the demo video."""
    return """    <model name="overview_camera">
      <static>true</static>
      <pose>%s</pose>
      <link name="link">
        <sensor name="overview_cam" type="camera">
          <always_on>true</always_on>
          <update_rate>20</update_rate>
          <camera>
            <horizontal_fov>0.9</horizontal_fov>
            <image><width>640</width><height>360</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>50</far></clip>
          </camera>
          <plugin name="overview_cam_controller" filename="libgazebo_ros_camera.so">
            <robotNamespace>/</robotNamespace>
            <alwaysOn>true</alwaysOn>
            <updateRate>0.0</updateRate>
            <cameraName>overview_cam</cameraName>
            <imageTopicName>image_raw</imageTopicName>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
            <frameName>overview_cam</frameName>
          </plugin>
        </sensor>
      </link>
    </model>
""" % OVERVIEW_POSE


def build_world(cfg, out_dir, with_overview_camera=False):
    cells = write_marker_assets(cfg, out_dir)
    marker = cfg['marker']
    half = float(marker['spacing_m']) / 2.0
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <world name="rail_entry">
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.8 0.85 1</background>
      <shadows>false</shadows>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>0.6 0.2 -1.0</direction>
    </light>
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>
        </visual>
      </link>
    </model>
%s%s%s%s    <gui>
      <camera name="overview">
        <pose>%s</pose>
      </camera>
    </gui>
  </world>
</sdf>
""" % (rail_model(cfg),
       marker_model(cfg, 'aruco_marker_left', half, int(marker['left_id']), cells, out_dir),
       marker_model(cfg, 'aruco_marker_right', -half, int(marker['right_id']), cells, out_dir),
       overview_camera() if with_overview_camera else '', OVERVIEW_POSE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--out', required=True, help='world file to write')
    parser.add_argument('--overview-camera', default='false', help='true adds the recording camera')
    args = parser.parse_args()
    with open(args.config) as handle:
        cfg = yaml.safe_load(handle)['rail_align']
    out_dir = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(out_dir, exist_ok=True)
    with open(args.out, 'w') as handle:
        handle.write(build_world(cfg, out_dir, args.overview_camera == 'true'))
    print(args.out)


if __name__ == '__main__':
    main()
