import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import csv
import random
import numpy.linalg as LA

import pickle as pkl

from planning_utils import a_star, heuristic, create_grid, prune_path
from planning_utils import create_grid_and_edges, graph_a_star
from planning_utils import prob_roadmap
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
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            # Calculate velocity
            vel = np.sqrt(self.local_velocity[0]**2 + self.local_velocity[1]**2)
            # Deadband in terms of velocity
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < np.sqrt(vel):
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            # Transition to disarming if it has stop moving
            # this considers landing on top of a building
            if self.local_velocity[2] < 0.01:
                self.disarming_transition()

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
        self.target_position = self.waypoints.pop(0)
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
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 20
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv', newline='') as f:
            reader = csv.reader(f)
            row1 = next(reader)

        lat0 = float(row1[0].split()[1])
        lon0 = float(row1[1].split()[1])

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # TODO: retrieve current global position
        global_pos = self.global_position

        # TODO: convert to current local position using global_to_local()
        local_pos = global_to_local(global_pos, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        #grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = (int(local_pos[0]-north_offset), int(local_pos[1]-east_offset), TARGET_ALTITUDE)
        # TODO: convert start position to current position rather than map center

        # Set goal as some arbitrary position on the grid
        while True:
            n_goal = random.randint(0,grid.shape[0])
            e_goal = random.randint(0,grid.shape[1])
            if grid[n_goal, e_goal] == 0:
                break
        #goal = global_to_local((-122.396585, 37.793520, TARGET_ALTITUDE), self.global_home)
        #grid_goal = (int(goal[0]-north_offset), int(goal[1]-east_offset))
        grid_goal = (n_goal, e_goal, TARGET_ALTITUDE)
        # TODO: adapt to set goal as latitude / longitude position and convert

        # Create probabilistic roadmap
        num_samples = 500
        z_max = TARGET_ALTITUDE
        G = prob_roadmap(data, num_samples, z_max)

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)

        # Find closest point in the graph to our current location
        graph_start = tuple(list(G.nodes())[np.argmin(list(map(lambda p: LA.norm(p-grid_start), np.array(list(G.nodes())))))])
        graph_goal = tuple(list(G.nodes())[np.argmin(list(map(lambda p: LA.norm(p-grid_goal), np.array(list(G.nodes())))))])

        print('Local Start and Goal: ', grid_start, grid_goal)
        print('Graph Start and Goal: ', graph_start, graph_goal)

        # Grid Search
        #path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # Graph Search
        path, _ = graph_a_star(G, heuristic, graph_start, graph_goal)
        # Append goal grid position to the path
        #path.append(grid_goal)

        # TODO: prune path to minimize number of waypoints
        path = prune_path(path)
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        print("Path: ", path)
        # Convert path to waypoints
        waypoints = [[int(p[0]) + north_offset, int(p[1]) + east_offset, TARGET_ALTITUDE, 0] for p in path]
        print("Waypoints: ", waypoints)
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

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
