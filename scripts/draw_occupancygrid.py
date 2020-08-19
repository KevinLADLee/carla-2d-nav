#!/usr/bin/env python

import glob
import os
import sys

import carla
import argparse
import logging

import numpy as np

import cv2 as cv

import yaml
import collections

# 10 cm/pix
# PIXELS_PER_METER = 10
PIXELS_PER_METER = 2

class CarlaGrid(object):
    def __init__(self, name, args, timeout):
        self.client = None
        self.name = name
        self.args = args
        self.timeout = timeout

        self.world = None
        self.town_map = None

        self._pixels_per_meter = PIXELS_PER_METER

        self._world_offset = (0, 0)

        self.waypoint_precision = 0.05
        self.patches = []

        self.max_x = 0
        self.min_x = 0
        self.max_y = 0
        self.min_y = 0

        self.height = 0
        self.width = 0

    def carla_transform_to_ros(self, location):
        location.y = -location.y
        return location

    def _get_data_from_carla(self):
        """Retrieves the data from the server side"""
        try:
            self.client = carla.Client(self.args.host, self.args.port)
            self.client.set_timeout(self.timeout)

            if self.args.map is None:
                world = self.client.get_world()
            else:
                world = self.client.load_world(self.args.map)
                print(self.args.map)

            town_map = world.get_map()
            return (world, town_map)

        except RuntimeError as ex:
            logging.error(ex)


    def start(self):
        self.world, self.town_map = self._get_data_from_carla()

        settings = self.world.get_settings()
        settings.no_rendering_mode = self.args.no_rendering
        self.world.apply_settings(settings)

        waypoints = self.town_map.generate_waypoints(2)
        margin = 50

        self.max_x = max(waypoints, key=lambda x: x.transform.location.x).transform.location.x + margin
        self.max_y = max(waypoints, key=lambda x: x.transform.location.y).transform.location.y + margin
        self.min_x = min(waypoints, key=lambda x: x.transform.location.x).transform.location.x - margin
        self.min_y = min(waypoints, key=lambda x: x.transform.location.y).transform.location.y - margin

        self._world_offset = (self.min_x, self.min_y)
        
        topology = self.town_map.get_topology()
        self.draw_topology(topology)
        self.generate_yaml()

    def world_to_pixel(self, location_x, location_y):
        """Converts the world coordinates to pixel coordinates"""
        x = self._pixels_per_meter * (location_x - self._world_offset[0])
        y = self._pixels_per_meter * (location_y - self._world_offset[1])
        return [int(x), int(y)]      
        
    def pixel_to_world(self, x, y):
        location_x = (x / self._pixels_per_meter) + self._world_offset[0]
        location_y = (y / self._pixels_per_meter) + self._world_offset[1]
        return [location_x, location_y]

    def lateral_shift(self, transform, shift):
        """Makes a lateral shift of the forward vector of a transform"""
        transform.rotation.yaw += 90
        return transform.location + shift * transform.get_forward_vector()

    def draw_topology(self, carla_topology):
        """ Draws traffic signs and the roads network with sidewalks, parking and shoulders by generating waypoints"""
        topology = [x[0] for x in carla_topology]
        topology = sorted(topology, key=lambda w: w.transform.location.z)
        set_waypoints = []
        for waypoint in topology:
            waypoints = [waypoint]
            # Generate waypoints of a road id. Stop when road id differs
            nxt = waypoint.next(self.waypoint_precision)
            if len(nxt) > 0:
                nxt = nxt[0]
                while nxt.road_id == waypoint.road_id:
                    waypoints.append(nxt)
                    nxt = nxt.next(self.waypoint_precision)
                    if len(nxt) > 0:
                        nxt = nxt[0]
                    else:
                        break
            set_waypoints.append(waypoints)
            for w in waypoints:
                # Classify lane types until there are no waypoints by going left
                l = w.get_left_lane()
                while l and l.lane_type != carla.LaneType.Driving:
                    l = l.get_left_lane()
                # Classify lane types until there are no waypoints by going right
                r = w.get_right_lane()
                while r and r.lane_type != carla.LaneType.Driving:
                    r = r.get_right_lane()
        
        # Draw Roads
        self.width  = self._pixels_per_meter * int(self.max_x-self.min_x)
        self.height = self._pixels_per_meter * int(self.max_y-self.min_y)
        img = np.zeros((self.height, self.width,1),np.uint8)
        
        for waypoints in set_waypoints:

            # waypoint = waypoints[0]

            road_left_side = [self.lateral_shift(w.transform, -w.lane_width * 0.5) for w in waypoints]
            road_right_side = [self.lateral_shift(w.transform, w.lane_width * 0.5) for w in waypoints]

            polygon = road_left_side + [x for x in reversed(road_right_side)]

            polygon = [self.world_to_pixel(x.x, x.y) for x in polygon]

            if len(polygon) > 2:
                poly_array = np.array(polygon, np.int32)
                poly_array.reshape((-1,1,2))
                cv.fillPoly(img, [poly_array], (255), lineType=cv.LINE_AA)
                # cv.polylines(img,[poly_array],True,(0,0,255))

        # init = self.world_to_pixel(0,0)
        # cv.circle(img, (init[0],init[1]), 5, (100), -1) 

        cv.imwrite("%s.png" % self.args.map, img) 
        # gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # cv.imwrite("%s_gray.png" % self.args.map, gray) 

    def generate_yaml(self):

        origin = self.pixel_to_world(0, self.height)
        print(origin)

        dict_file = {
                     'image' : '%s.png' % self.args.map,
                     'resolution' : 1.0 / PIXELS_PER_METER,
                     'origin' : [origin[0], -origin[1],0],
                     'negate': 0,
                     'occupied_thresh': 0.65,
                     'free_thresh': 0.2
                    }          
        with open('%s.yaml' % self.args.map, 'w') as file:
            yaml.dump(dict_file, file)
            # For python yaml 5.0 and later
            # documents = yaml.dump(dict_file, file, sort_keys=False)

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Map to Grid')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--map',
        metavar='TOWN',
        default=None,
        help='start a new episode at the given TOWN')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        help='switch off server rendering')  

    args = argparser.parse_args()
    args.description = argparser.description

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)
    logging.info('listening to server %s:%s', args.host, args.port)
    print(__doc__)
    world = CarlaGrid("CarlaGrid", args, timeout=2.0)
    world.start()

if __name__ == '__main__':
    main()







