#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from teknofest_pkg.visualizer import Visualizer


class SegmentModel:
    '''
        Klasse repräsentiert eine Segment Linie.
    '''

    def __init__(self, y_offset, point_distance):
        '''
            Konstruktor zum Instanziieren einer Segment Linie.

            Parameter
            ---------
            y_offset : Integer
                y-Wert an dem das Segment liegen soll.
            point_distance : Integer
                Abstand zwischen dem linken und rechten Punkt (Straßenmarkierung) auf dem Segment.

        '''
        self.y_offset = y_offset
        self.nz_left_points = []
        self.nz_right_points = []
        self.point_center = None
        self.left_point = None
        self.right_point = None
        self.point_distance = point_distance
        self.vis = Visualizer()

    def draw(self, image, thickness=1):
        '''
            Methode zeichnet alle wichtigen Eigenschaften (Straßenmarkierung, Mittelpunkt,
            y-Linie) des Segments auf ein Bild.

            Parameter
            ---------
            image : Bild
            thickness (optional) : Integer
                Dicke der gezeichneten Linien auf dem Bild.
                Default Wert ist 1.


        '''
        height, width, _ = image.shape
        self.vis.draw_line(image, (0, self.y_offset), (width, self.y_offset), (255, 0, 0), thickness)
        self.vis.draw_line(image, (int(width/2), 0), (int(width/2), height), (0, 0, 255), thickness)
        if self.left_point:
            self.vis.draw_point(image, (int(self.left_point), self.y_offset), 3, (0, 255, 0), thickness)
        if self.right_point:
            self.vis.draw_point(image, (int(self.right_point), self.y_offset), 3, (0, 255, 0), thickness)
        if self.point_center:
            self.vis.draw_point(image, (int(self.point_center), self.y_offset), 3, (0, 0, 255), thickness)

    def update_non_zero_points(self, image):
        '''
            Methode berechnet alle weißen Pixel (non zero), welche auf dem Segment liegen
            und teilt diese in eine linke und rechte Liste auf.

            Parameter
            ---------
            image : Bild
        '''
        self.nz_left_points, self.nz_right_points = self._calc_non_zero(image)

    def update_point_distance(self):
        '''
            Methode berechnet und aktualisiert den Abstand zwischen rechten und linken Punkt (Straßenmarkierung)
            für dieses Segment.
        '''
        if self.left_point and self.right_point:
            new_distance = self.right_point - self.left_point
            self.point_distance = new_distance

    def update_point_center(self):
        '''
            Methode berechnet und aktualisiert den Mittelpunkt der Straße für dieses Segment.
        '''
        if self.left_point and self.right_point:
            self.point_center = int((self.left_point + self.right_point) / 2)

    def _calc_non_zero(self, image):
        if self.point_center:
            separator = self.point_center
        else:
            _, width, _ = image.shape
            separator = int(width / 2)
            self.point_center = separator
        arr = np.array(np.nonzero(image[self.y_offset])[0])
        nz_lp, nz_rp = [arr[arr < separator], arr[~(arr < separator)]]
        nz_lp, nz_rp = np.unique(nz_lp), np.unique(nz_rp)  # entferne doppelte Werte
        nz_lp = nz_lp[::-1]  # invertiere liste
        return nz_lp, nz_rp

    def __str__(self):
        ret = "SegmentModel {\n"
        ret += "\ty_offset : " + str(self.y_offset) + "\n"
        ret += "\tnz_left_points : " + np.array_str(self.nz_left_points) + "\n"
        ret += "\tnz_right_points : " + np.array_str(self.nz_right_points) + "\n"
        ret += "\tleft_point : " + str(self.left_point) + "\n"
        ret += "\tright_point : " + str(self.right_point) + "\n"
        ret += "\tpoint_center : " + str(self.point_center) + "\n"
        ret += "}\n"
        return ret
