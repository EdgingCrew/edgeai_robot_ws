#!/usr/bin/env python3
"""Generate the ArUco target stand models models/aruco_stand_<id>/ (run once, output is committed).

Each stand is a small base, a 0.05 m square pole (so the lidar sees it) and a 0.22 m white board
centred 0.35 m above the floor, with a DICT_4X4_50 marker of 0.16 m side (midterm rules) on its
front face. Model origin = ground point beneath the stand (the target reference position); the
marker faces +x. Worlds place a stand with <include><uri>model://aruco_stand_<id></uri>.

Usage: python3 make_aruco_stands.py [--ids 1-12]
"""
import argparse
import os

import cv2
import numpy as np

MARKER_M = 0.16
BOARD_M = 0.22
PX_PER_M = 2000  # 0.16 m -> 320 px, a multiple of the 6-cell marker (4x4 + black border)
MODELS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models'))

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>aruco_stand_{id}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Freestanding target stand with ArUco DICT_4X4_50 marker ID {id} (0.16 m, centre 0.35 m above
the floor). Model origin = ground point beneath the stand (the target reference position);
the marker faces +x.</description>
</model>
"""

MODEL_SDF = """<?xml version="1.0" ?>
<sdf version="1.9">
  <!-- Origin: ground point beneath the stand (target reference position). Marker faces +x. -->
  <model name="aruco_stand_{id}">
    <static>true</static>
    <link name="link">
      <!-- Base plate -->
      <collision name="base_collision">
        <pose>0 0 0.01 0 0 0</pose>
        <geometry><box><size>0.08 0.08 0.02</size></box></geometry>
      </collision>
      <visual name="base_visual">
        <pose>0 0 0.01 0 0 0</pose>
        <geometry><box><size>0.08 0.08 0.02</size></box></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
      </visual>
      <!-- Pole (0.05 m square so the lidar sees it) -->
      <collision name="pole_collision">
        <pose>0 0 0.24 0 0 0</pose>
        <geometry><box><size>0.05 0.05 0.44</size></box></geometry>
      </collision>
      <visual name="pole_visual">
        <pose>0 0 0.24 0 0 0</pose>
        <geometry><box><size>0.05 0.05 0.44</size></box></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
      </visual>
      <!-- White backing board, 0.22 m square, centre 0.35 m above the floor -->
      <collision name="board_collision">
        <pose>0.0275 0 0.35 0 0 0</pose>
        <geometry><box><size>0.005 0.22 0.22</size></box></geometry>
      </collision>
      <visual name="board_visual">
        <pose>0.0275 0 0.35 0 0 0</pose>
        <geometry><box><size>0.005 0.22 0.22</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
      </visual>
      <!-- Marker texture (0.16 m marker + white margin) on a thin plate in front of the board -->
      <visual name="marker_visual">
        <pose>0.0305 0 0.35 0 0 0</pose>
        <geometry><box><size>0.001 0.22 0.22</size></box></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <pbr>
            <metal>
              <albedo_map>model://aruco_stand_{id}/materials/textures/aruco_{id}.png</albedo_map>
              <metalness>0.0</metalness>
              <roughness>1.0</roughness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def marker_texture(marker_id):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    side = int(round(MARKER_M * PX_PER_M))
    marker = cv2.aruco.generateImageMarker(dictionary, marker_id, side)
    board = int(round(BOARD_M * PX_PER_M))
    img = np.full((board, board), 255, dtype=np.uint8)
    off = (board - side) // 2
    img[off:off + side, off:off + side] = marker
    return img


def write_stand(marker_id):
    model_dir = os.path.join(MODELS, f'aruco_stand_{marker_id}')
    tex_dir = os.path.join(model_dir, 'materials', 'textures')
    os.makedirs(tex_dir, exist_ok=True)
    with open(os.path.join(model_dir, 'model.config'), 'w') as f:
        f.write(MODEL_CONFIG.format(id=marker_id))
    with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
        f.write(MODEL_SDF.format(id=marker_id))
    cv2.imwrite(os.path.join(tex_dir, f'aruco_{marker_id}.png'), marker_texture(marker_id))
    return model_dir


def parse_ids(text):
    ids = set()
    for part in text.split(','):
        lo, _, hi = part.partition('-')
        ids.update(range(int(lo), int(hi or lo) + 1))
    if not ids or min(ids) < 0 or max(ids) > 49:
        raise argparse.ArgumentTypeError('DICT_4X4_50 has marker ids 0-49')
    return sorted(ids)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ids', type=parse_ids, default=parse_ids('1-12'), help='e.g. 1-12 or 1,2,7')
    for marker_id in ap.parse_args().ids:
        print('wrote', write_stand(marker_id))


if __name__ == '__main__':
    main()
