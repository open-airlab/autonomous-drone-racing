#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 transpalette <transpalette@arch-cactus>
#
# Distributed under terms of the MIT license.

"""
Dataset class, holding background images along with their annotations
"""

import os
import random

from pyrr import Vector3
from queue import Queue
from tqdm import tqdm
from PIL import Image


class BackgroundAnnotations:
    def __init__(self, height: float, roll: float, pitch: float, yaw: float):
        self.height = height
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


'''
Holds a background image along with its annotations
'''
class BackgroundImage:
    def __init__(self, image_path: str, annotations: BackgroundAnnotations):
        self.file = image_path
        self.annotations = annotations

    def image(self):
        return Image.open(self.file)


class SyntheticAnnotations:
    def __init__(self, center: Vector3, orientation: Vector3, on_screen: bool):
        self.center = center
        self.orientation = orientation
        self.on_screen = on_screen


'''
Holds a generated image along with its annotations
'''
class AnnotatedImage:
    def __init__(self, image: Image, id, annotations: SyntheticAnnotations):
        self.image = image
        self.id = id
        self.annotations = annotations


class Dataset:
    def __init__(self, path: str):
        if not os.path.isdir(path):
            raise Exception("Dataset directory not found")
        self.path = path
        self.width = None
        self.height = None
        self.data = Queue()

    def parse_annotations(self, path: str):
        if not os.path.isfile(path):
            raise Exception("Annotations file not found")
        # Example:
        annotations = {
            '0001.jpg':
                {'height': 100, 'roll': 0, 'pitch': 0, 'yaw': 0}
        }
        with open(path) as file:
            # TODO
            pass

        return annotations

    def load(self, annotations_path=None):
        print("[*] Loading and randomizing base dataset...")
        files = os.listdir(self.path)
        random.shuffle(files)
        annotations = self.parse_annotations(annotations_path)
        for file in tqdm(files):
            full_path = os.path.join(self.path, file)
            if os.path.isfile(full_path) and full_path != annotations_path:
                self.data.put(BackgroundImage(full_path, annotations['0001.jpg']))
                if not self.width and not self.height:
                    with Image.open(full_path) as img:
                        self.width, self.height = img.size

    '''
    Returns the next BackgroundImage in the Queue
    '''
    def get(self):
        return self.data.get()

    def put(self, image: AnnotatedImage):
        self.data.put(image)

    def save(self):
        for annotatedImage in tqdm(iter(self.data.get, object())):
            annotatedImage.image.save(
                os.path.join(self.path + str(annotatedImage.id) + '.png')
            )
            if self.data.empty():
                break

    def get_image_size(self):
        print("[*] Using {}x{} resolution".format(self.width, self.height))
        return (self.width, self.height)