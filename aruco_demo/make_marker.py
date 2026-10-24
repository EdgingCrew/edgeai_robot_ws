#!/usr/bin/env python3
import argparse
import os
import cv2

def parse_args():
    parser = argparse.ArgumentParser(description='Generate an ArUco marker image')
    parser.add_argument('--id', type=int, default=0, help='marker id')
    parser.add_argument('--dict', default='DICT_4X4_50', help='name of the cv2.aruco dictionary')
    parser.add_argument('--pixels', type=int, default=400, help='marker side in pixels')
    parser.add_argument('--margin', type=int, default=100, help='white margin in pixels')
    parser.add_argument('--out', default='~/robot_scratch/aruco', help='output directory')
    return parser.parse_args()


def main():
    args = parse_args()

    # Look up the dictionary constant by its name, e.g. 'DICT_4X4_50'
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dict))

    # Marker image: black frame plus the id bits, as a numpy array
    marker = cv2.aruco.generateImageMarker(dictionary, args.id, args.pixels)

    # The detector needs a light border around the black frame (quiet zone)
    m = args.margin
    padded = cv2.copyMakeBorder(marker, m, m, m, m, cv2.BORDER_CONSTANT, value=255)

    out_dir = os.path.expanduser(args.out)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f'aruco_{args.id}.png')
    cv2.imwrite(path, padded)
    print(f'Saved {path} ({padded.shape[1]}x{padded.shape[0]} px)')


if __name__ == '__main__':
    main()
