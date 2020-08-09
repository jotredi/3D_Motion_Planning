import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import csv
import random
import numpy.linalg as LA

from planning_utils import a_star, heuristic, create_grid, prune_path
from utils_3D import create_voxmap, a_star_3D

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.global_waypoints = []
        self.local_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # 1m3 voxmap
        self.voxmap = []
        self.north_offset = 0
        self.east_offset = 0

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.plan_local_path()
                self.waypoint_transition()
                #self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            # Calculate velocity
            vel = np.sqrt(self.local_velocity[0]**2 + self.local_velocity[1]**2)

            if len(self.global_waypoints) and len(self.local_waypoints) < 5:
                self.plan_local_path()

            # Deadband in terms of velocity
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < np.sqrt(1.5*vel):
                if len(self.local_waypoints):
                    self.waypoint_transition()
                #if len(self.global_waypoints) > 0:
                #    if len(self.local_waypoints) > 0:
                #        self.waypoint_transition()
                #    else:
                #        self.plan_local_path()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()
            # Transition to disarming if it has stop moving
            # this considers landing on top of a building
            #if self.local_velocity[2] < 0.001:
            #    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.local_waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.global_waypoints)
        self.connection._master.write(data)

    def send_local_waypoints(self):
        print("Sending local waypoints to simulator ...")
        data = msgpack.dumps(self.local_waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 20
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # Read lat0, lon0 from colliders into floating point values
        with open('colliders.csv', newline='') as f:
            reader = csv.reader(f)
            row1 = next(reader)

        lat0 = float(row1[0].split()[1])
        lon0 = float(row1[1].split()[1])

        # Set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # Retrieve current global position
        global_pos = self.global_position

        # Convert to current local position using global_to_local()
        local_pos = global_to_local(global_pos, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        #grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

        # Create a voxmap (2.5D map) centered at TARGET_ALTITUDE
        # and with a resolution of:
        VOXMAP_RES = 10
        voxmap, north_offset, east_offset = create_voxmap(data, TARGET_ALTITUDE, SAFETY_DISTANCE, VOXMAP_RES)
        # Create a 1m3 resolution voxmap
        self.voxmap, self.north_offset, self.east_offset = create_voxmap(data, TARGET_ALTITUDE, SAFETY_DISTANCE, 1)

        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid
        grid_start = (int(local_pos[0]-north_offset), int(local_pos[1]-east_offset), TARGET_ALTITUDE)

        # Define starting point on the voxmap
        voxmap_start = (grid_start[0] // VOXMAP_RES, grid_start[1] // VOXMAP_RES, grid_start[2] // VOXMAP_RES)

        # Set goal as some arbitrary position on the grid/voxmap
        while True:
            n_goal = random.randint(0, voxmap.shape[0] - 1)
            e_goal = random.randint(0, voxmap.shape[1] - 1)
            alt_goal = random.randint(0, voxmap.shape[2] - 1)
            if voxmap[n_goal, e_goal, alt_goal] == 0:
                break

        #goal = global_to_local((-122.396585, 37.793520, TARGET_ALTITUDE), self.global_home)
        voxmap_goal = (n_goal, e_goal, alt_goal)

        # Run A* to find a path from start to goal

        #print('Local Start and Goal: ', grid_start, grid_goal)
        print('Voxmap Start and Goal: ', voxmap_start, voxmap_goal)

        # Grid Search
        #path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # Graph Search
        #path, _ = graph_a_star(G, heuristic, graph_start, graph_goal)

        # 3D A* Search
        path, _ = a_star_3D(voxmap, heuristic, voxmap_start, voxmap_goal)

        # Prune path to minimize number of waypoints
        path = prune_path(path)
        print("Path: ", path)

        # Convert path to waypoints
        waypoints = []
        for i in range(len(path)):
            p = path[i]
            if i:
                last_p = path[i-1]
                # Set heading based on relative position to last wp
                heading = np.arctan2((p[1]-last_p[1]), (p[0]-last_p[0]))
            else:
                heading = 0
            # Append waypoint
            waypoints.append([p[0] * VOXMAP_RES + north_offset, p[1] * VOXMAP_RES + east_offset, p[2] * VOXMAP_RES, heading])

        #waypoints = [[p[0] * VOXMAP_RES + north_offset, p[1] * VOXMAP_RES + east_offset, p[2] * VOXMAP_RES, 0] for p in path]
        print("Waypoints: ", waypoints)

        # Set self.global_waypoints
        self.global_waypoints = waypoints
        # Send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def plan_local_path(self):
        "Plans a local path inside a local volume around the current location"

        # Define current position in the (global) voxmap
        if len(self.local_waypoints):
            last_wp = self.local_waypoints[-1]
            current_voxmap = (int(last_wp[0]-self.north_offset),
                              int(last_wp[1]-self.east_offset),
                              int(last_wp[2]))
        else:
            current_voxmap = (int(self.local_position[0]-self.north_offset),
                              int(self.local_position[1]-self.east_offset),
                              int(-self.local_position[2]))

        # Define size of local map (40m x 40m x 10 m)
        d_north = 20
        d_east = 20
        d_alt = 5

        vox_shape = self.voxmap.shape

        north_min = int(np.clip(current_voxmap[0] - d_north, 0, vox_shape[0]))
        north_max = int(np.clip(current_voxmap[0] + d_north, 0, vox_shape[0]))

        east_min = int(np.clip(current_voxmap[1] - d_east, 0, vox_shape[1]))
        east_max = int(np.clip(current_voxmap[1] + d_east, 0, vox_shape[1]))

        alt_min = int(np.clip(current_voxmap[2] - d_alt, 0, vox_shape[2]))
        alt_max = int(np.clip(current_voxmap[2] + d_alt, 0, vox_shape[2]))

        local_voxmap = self.voxmap[north_min:north_max, east_min:east_max, alt_min:alt_max]

        # Define next global wp
        next_wp = self.global_waypoints[0]

        # Define start in the center of voxmap
        local_start = (d_north, d_east, d_alt)

        # Define goal to a node in the direction of the next waypoint
        local_goal = (np.clip(next_wp[0]-self.north_offset-north_min, 0, 2*d_north-1),
                      np.clip(next_wp[1]-self.east_offset-east_min, 0, 2*d_east-1),
                      np.clip(next_wp[2]-alt_min, 0, 2*d_alt-1))

        # 3D A* Search
        path, _ = a_star_3D(local_voxmap, heuristic, local_start, local_goal)

        # Prune path to minimize number of waypoints
        path = prune_path(path)

        # Convert path to waypoints
        for i in range(1, len(path)):
            p = path[i]
            # Append waypoint to local waypoints
            self.local_waypoints.append([p[0] + self.north_offset + north_min,
                                         p[1] + self.east_offset + east_min,
                                         p[2] + alt_min, next_wp[3]])

            # Delete next global wp if already reached
            if np.linalg.norm(np.array(self.local_waypoints[-1]) - np.array(self.global_waypoints[0])) < 1.0:
                del self.global_waypoints[0]

        print("Waypoints: ", self.local_waypoints)
        # Send waypoints to sim (this is just for visualization of waypoints)
        #self.send_local_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=120)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
