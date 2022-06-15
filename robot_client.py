#!/usr/bin/env python3

from robots import robots

import asyncio
import websockets
import json
import signal
import time
import sys
from enum import Enum
import time
import random

import colorama
from colorama import Fore

colorama.init(autoreset=True)

# Persistent Websockets!!!!!!!!!!!!!!!!
# https://stackoverflow.com/questions/59182741/python-websockets-lib-client-persistent-connection-with-class-implementation

# Handle Ctrl+C termination

# https://stackoverflow.com/questions/2148888/python-trap-all-signals
SIGNALS_TO_NAMES_DICT = dict((getattr(signal, n), n) \
    for n in dir(signal) if n.startswith('SIG') and '_' not in n)

# https://github.com/aaugustin/websockets/issues/124
__kill_now = False


def __set_kill_now(signum, frame):
    print('\nReceived signal:', SIGNALS_TO_NAMES_DICT[signum], str(signum))
    global __kill_now
    __kill_now = True


signal.signal(signal.SIGINT, __set_kill_now)
signal.signal(signal.SIGTERM, __set_kill_now)


def kill_now() -> bool:
    global __kill_now
    return __kill_now

if len(sys.argv) > 1 and sys.argv[1] == "--simulator":
    server_address = sys.argv[2]
    server_port = sys.argv[3]
    robot_port = sys.argv[3]
else:
    server_address = "144.32.165.233"
    server_port = 80
    robot_port = 80

server_connection = None
active_robots = {}
ids = []


class RobotState(Enum):
    FORWARDS = 1
    BACKWARDS = 2
    LEFT = 3
    RIGHT = 4
    STOP = 5


class Robot:

    BAT_LOW_VOLTAGE = 3.6
    MAX_SPEED = 100

    def __init__(self, id):
        self.id = id
        self.connection = None

        self.orientation = 0
        self.neighbours = {}

        self.teleop = False
        self.state = RobotState.STOP
        self.ir_readings = []
        self.battery_charging = False
        self.battery_voltage = 0
        self.battery_percentage = 0
        self.in_task = False
        self.task_id = -1

        self.random_left = self.MAX_SPEED
        self.random_right = self.MAX_SPEED * (0.5 + (random.random()*0.5))

        self.turn_time = time.time()

        if id < 31:
            self.ir_threshold = 200  # Pi-puck
        else:
            self.ir_threshold = 80  # Mona


async def connect_to_server():
    uri = f"ws://{server_address}:{server_port}"
    connection = await websockets.connect(uri, ping_interval=None)

    print("Opening connection to server: " + uri)

    awake = await check_awake(connection)

    if awake:
        print("Server is awake")
        global server_connection
        server_connection = connection
    else:
        print("Server did not respond")


async def connect_to_robots():
    for id in active_robots.keys():
        ip = robots[id]
        if ip != '':
            uri = f"ws://{server_address}:{server_port}"
            connection = await websockets.connect(uri, ping_interval=None)

            print("Opening connection to robot:", uri)

            awake = await check_awake(connection)

            if awake:
                print(f"Robot {id} is awake")

                if len(sys.argv) > 1 and sys.argv[1] == "--simulator":
                    await request_robot(connection, id)

                active_robots[id].connection = connection
            else:
                print(f"Robot {id} did not respond")
        else:
            print(f"No IP defined for robot {id}")

# Simulator-only robot request
async def request_robot(connection, robotID):
    try:
        message = {"sel_robot": robotID}

        # Send request for data and wait for reply
        await connection.send(json.dumps(message))
        reply_json = await connection.recv()
        reply = json.loads(reply_json)
        print("Got response from robot", robotID)
        return reply

    except Exception as e:
        print(f"{type(e).__name__}: {e}")

async def check_awake(connection):
    awake = False

    try:
        message = {"check_awake": True}

        # Send request for data and wait for reply
        await connection.send(json.dumps(message))
        reply_json = await connection.recv()
        reply = json.loads(reply_json)

        awake = reply["awake"]

    except Exception as e:
        print(f"check_awake {type(e).__name__}: {e}")

    return awake


async def get_robot_data(ids):
    await message_robots(ids, get_data)


async def send_robot_commands(ids):
    await message_robots(ids, send_commands)


async def stop_robots(ids):
    await message_robots(ids, stop_robot)


# https://stackoverflow.com/questions/49858021/listen-to-multiple-socket-with-websockets-and-asyncio
async def message_robots(ids, function):
    loop = asyncio.get_event_loop()

    tasks = []
    for id, robot in active_robots.items():
        if id in ids:
            tasks.append(loop.create_task(function(robot)))

    await asyncio.gather(*tasks)


async def get_server_data():
    try:
        global ids
        message = {"get_robots": True}

        # Send request for data and wait for reply
        await server_connection.send(json.dumps(message))
        reply_json = await server_connection.recv()
        reply = json.loads(reply_json)

        ids = list(reply.keys())
        ids = [int(id) for id in ids]

        for id, robot in reply.items():
            id = int(id)  # ID is sent as an integer - why is this necessary?
            if id in active_robots.keys():  # Filter based on robots of interest

                active_robots[id].orientation = robot["orientation"]
                active_robots[id].neighbours = {k:r for k,r in robot["neighbours"].items() if int(k) in active_robots}
                active_robots[id].tasks = robot["tasks"]

                print(f"Robot {id}")
                print(f"Orientation = {active_robots[id].orientation}")
                print(f"Neighbours = {active_robots[id].neighbours}")
                print(f"Tasks = {active_robots[id].tasks}")
                print()

    except Exception as e:
        print(f"get_server_data {type(e).__name__}: {e}")


async def stop_robot(robot):
    try:
        # Turn off LEDs and motors when killed
        message = {"set_leds_colour": "off", "set_motor_speeds": {}}
        message["set_motor_speeds"]["left"] = 0
        message["set_motor_speeds"]["right"] = 0
        await robot.connection.send(json.dumps(message))

        # Send command message
        await robot.connection.send(json.dumps(message))
    except Exception as e:
        print(f"stop_robot {type(e).__name__}: {e}")


async def get_data(robot):
    # try:
        message = {"get_ir": True, "get_battery": True}

        # Send request for data and wait for reply
        await robot.connection.send(json.dumps(message))
        reply_json = await robot.connection.recv()
        reply = json.loads(reply_json)

        robot.ir_readings = reply["ir"]


          # robot.battery_voltage = reply["battery"]["voltage"]
          # robot.battery_percentage = reply["battery"]["percentage"]

        print(f"[Robot {robot.id}] IR readings: {robot.ir_readings}")
        # print("[Robot {}] Battery: {:.2f}V, {}%" .format(robot.id,
        #                                                  robot.battery_voltage,
        #                                                  robot.battery_percentage))

    # except Exception as e:
    #     print(f"get_ir {type(e).__name__}: {e}")


async def send_commands(robot):
    try:
        # Turn off LEDs and motors when killed
        if kill_now():
            message = {"set_leds_colour": "off", "set_motor_speeds": {}}
            message["set_motor_speeds"]["left"] = 0
            message["set_motor_speeds"]["right"] = 0
            await robot.connection.send(json.dumps(message))

        # Construct command message
        message = {}

        if robot.teleop:
            message["set_leds_colour"] = "blue"
            if robot.state == RobotState.FORWARDS:
                left = right = robot.MAX_SPEED
            elif robot.state == RobotState.BACKWARDS:
                left = right = -robot.MAX_SPEED
            elif robot.state == RobotState.LEFT:
                left = -robot.MAX_SPEED * 0.6
                right = robot.MAX_SPEED * 0.6
            elif robot.state == RobotState.RIGHT:
                left = robot.MAX_SPEED * 0.6
                right = -robot.MAX_SPEED * 0.6
            elif robot.state == RobotState.STOP:
                left = right = 0
        else:
          left = right = 0
          leaderRobot = -1
          goToFriend = [-1, -1]
          test_for_task = 0

          for taskID in robot.tasks:
            if (robot.tasks[taskID]["range"] < 0.25):
              test_for_task += 1

          if test_for_task == 0:
            robot.in_task = False
            robot.task_id = -1

          # Autonomous mode
          for key, activeRobot in active_robots.items():
            # Identify an active tele-operated robot
            if (activeRobot.teleop and str(key) in robot.neighbours.keys()):
              leaderRobot = activeRobot.id
            # Identify a neighbour robot within a task
            if (activeRobot.in_task and str(key) in robot.neighbours.keys()):
              goToFriend[0] = activeRobot.id
              goToFriend[1] = activeRobot.task_id
          # If there is a tele operated robot, follow it
          if (leaderRobot != -1):
            message["set_leds_colour"] = "magenta"
            left, right = head_towards_leader(robot, active_robots[leaderRobot], left, right)
          else:
            # If a robot can see a neighbours task, go to it
            if (goToFriend[1] in robot.tasks.keys()): #If Robot can see task, go towards it
              message["set_leds_colour"] = "yellow"
              left, right = head_towards_goal(robot, left, right, active_robots[goToFriend[0]].task_id)
            # If a robot can see a neighbour in a task they cannot see, go to it
            elif (str(goToFriend[0]) in robot.neighbours.keys()):
              message["set_leds_colour"] = "cyan"
              left, right = head_towards_leader(robot, active_robots[goToFriend[0]], left, right)
            else:
              # Otherwise if they have no neighbours in goals, head towards their own goal
              # If there are no goals, this will result in random sweeping movement
              message["set_leds_colour"] = "black"
              left, right = head_towards_goal(robot, left, right)
              if (left != 0 and right != 0):
                # If the robot is intending to move, run object avoidance
                left, right = object_avoidance(robot, left, right)


        message["set_motor_speeds"] = {}
        message["set_motor_speeds"]["left"] = left
        message["set_motor_speeds"]["right"] = right

        # Set Pi-puck RGB LEDs based on battery voltage
        if robot.battery_voltage < robot.BAT_LOW_VOLTAGE:
            message["set_leds_colour"] = "red"
        else:
            message["set_leds_colour"] = "green"

        # Send command message
        # print("Sending command message to", robot)
        await robot.connection.send(json.dumps(message))

    except ValueError as e:
        print(f"send_commands {type(e).__name__}: {e}")


def getSmallestAngle(desired, actual):
  diff = desired - actual
  if (abs(diff) > 180):
    if diff < 0:
      heading = 360 - abs(diff)
    else:
      heading = -(360 - abs(diff))
  else:
    heading = diff
  return heading


def steer():
  pass


def head_towards_leader(robot, control_robot, left, right):
  if (robot.neighbours[str(control_robot.id)]["range"] < 0.10):
    new_heading = getSmallestAngle(control_robot.orientation, robot.orientation)
  else:
    desired_bearing = robot.neighbours[str(control_robot.id)]["bearing"]
    new_heading = desired_bearing

  coeff = abs(new_heading/180)

  print(desired_bearing)
  print(robot.orientation)
  print(new_heading)

  if (new_heading > 20):
    left = robot.MAX_SPEED
    right = robot.MAX_SPEED * (1-coeff)
  elif (new_heading < -20):
    #left = new_heading / robot.MAX_SPEED
    left = robot.MAX_SPEED * (1-coeff)
    right = robot.MAX_SPEED

  return left, right

def object_avoidance(robot, left, right):
  if robot.ir_readings == {}:
    return left, right
  elif robot.ir_readings[1] > robot.ir_threshold and robot.ir_readings[6] > robot.ir_threshold and robot.ir_readings[3] > robot.ir_threshold and robot.ir_readings[4] > robot.ir_threshold:
    #Robot is trapped. Need to program behaviour for this
    pass
  elif robot.ir_readings[0] > robot.ir_threshold or robot.ir_readings[1] > robot.ir_threshold:
    left = -robot.MAX_SPEED/2
    right = robot.MAX_SPEED/2
  elif robot.ir_readings[6] > robot.ir_threshold or robot.ir_readings[7] > robot.ir_threshold:
    left = robot.MAX_SPEED/2
    right = -robot.MAX_SPEED/2
  #If there is no objects to collide into, the initial decision for left and right is adhered to
  return left, right


def head_towards_goal(robot, left, right, task_to_go_to=-1):
  selected_task_ID = -1 #
  closest_task = 1 #1m is greater than sensing range and as such any task will be closer than this
  if robot.tasks == {}: # No tasks found, perform random walk
    return random_walk(robot, left, right)
  if task_to_go_to != -1:
    selected_task_ID = task_to_go_to
  else:
    try:
      for taskID in robot.tasks:
        if robot.tasks[taskID]["range"] < closest_task:
          selected_task_ID = taskID
          closest_task = robot.tasks[taskID]["range"]
    except Exception as e:
      print(f"An error has occured unpacking the tasks range. Error was: {e}")

  # If task is within 10cm, stop moving (TODO: Decide if waiting is worthwhile)
  if robot.tasks[selected_task_ID]["range"] < 0.10:
    left = right = 0
    robot.in_task = True
    robot.task_id = selected_task_ID
    return left, right
  elif robot.tasks[selected_task_ID]["range"] < 0.25:
    robot.in_task = True
    robot.task_id = selected_task_ID

  #Rotate clockwise
  if robot.tasks[selected_task_ID]['bearing'] > 20:
    left = robot.MAX_SPEED / 3
    right = -robot.MAX_SPEED / 3
  #Rotate anti-clockwise
  elif robot.tasks[selected_task_ID]['bearing'] < -20:
    left = -robot.MAX_SPEED / 3
    right = robot.MAX_SPEED / 3
  #Else head forwards
  else:
    left = right = robot.MAX_SPEED

  return left, right

def aggregate(robot):
  """
  Basic aggregation algorithm, WIP
  """
  av_bearing = 0
  left = 0
  right = 0
  for neighbour in robot.neighbours:
    av_bearing += robot.neighbours[neighbour]["bearing"]
  if (av_bearing > -30 and av_bearing < 30):
    left = right = robot.MAX_SPEED
  elif (av_bearing <= -30):
    left = robot.MAX_SPEED / 4
    right = -robot.MAX_SPEED / 4
  elif (av_bearing >= 30):
    left = -robot.MAX_SPEED / 4
    right = robot.MAX_SPEED / 4
  else:
    left = right = robot.MAX_SPEED
  return left, right


def random_walk(robot, left, right):
  if random.random() > 0.80:
    if random.choice([True,False]):
      robot.random_left = robot.MAX_SPEED
      robot.random_right = robot.MAX_SPEED * (0.5 + (random.random()*0.5))
    else:
      robot.random_right = robot.MAX_SPEED
      robot.random_left = robot.MAX_SPEED * (0.5 + (random.random()*0.5))
  return robot.random_left, robot.random_right

def default_behaviour(robot):
  if robot.state == RobotState.FORWARDS:
      left = right = robot.MAX_SPEED
      if (time.time() - robot.turn_time > 0.5) and any(ir > robot.ir_threshold for ir in robot.ir_readings):
          robot.turn_time = time.time()
          robot.state = random.choice((RobotState.LEFT, RobotState.RIGHT))
  elif robot.state == RobotState.BACKWARDS:
      left = right = -robot.MAX_SPEED
      robot.turn_time = time.time()
      robot.state = RobotState.FORWARDS
  elif robot.state == RobotState.LEFT:
      left = -robot.MAX_SPEED
      right = robot.MAX_SPEED
      if time.time() - robot.turn_time > random.uniform(0.5, 1.0):
          robot.turn_time = time.time()
          robot.state = RobotState.FORWARDS
  elif robot.state == RobotState.RIGHT:
      left = robot.MAX_SPEED
      right = -robot.MAX_SPEED
      if time.time() - robot.turn_time > random.uniform(0.5, 1.0):
          robot.turn_time = time.time()
          robot.state = RobotState.FORWARDS
  elif robot.state == RobotState.STOP:
      left = right = 0
      robot.turn_time = time.time()
      robot.state = RobotState.FORWARDS

  return left,right


class MenuState(Enum):
    START = 1
    SELECT = 2
    DRIVE = 3


async def send_message(websocket, message):
    await websocket.send(json.dumps({"prompt": message}))


async def handler(websocket):
    state = MenuState.START
    robot_id = ""
    valid_robots = list(active_robots.keys())
    forwards = "w"
    backwards = "s"
    left = "a"
    right = "d"
    stop = " "
    release = "q"

    async for packet in websocket:
        message = json.loads(packet)
        # print(message)

        if "key" in message:

            key = message["key"]

            if key == "teleop_start":
                state = MenuState.START

            if key == "teleop_stop":
                if state == MenuState.DRIVE:
                    id = int(robot_id)
                    active_robots[id].teleop = False
                    active_robots[id].state = RobotState.STOP

            if state == MenuState.START:
                await send_message(websocket, f"\r\nEnter robot ID ({valid_robots}), then press return: ")
                robot_id = ""
                state = MenuState.SELECT

            elif state == MenuState.SELECT:
                if key == "\r":
                    valid = False
                    try:
                        if int(robot_id) in valid_robots:
                            valid = True
                            await send_message(websocket, f"\r\nControlling robot ({release} to release): " + robot_id)
                            await send_message(websocket, f"\r\nControls: Forwards = {forwards}; Backwards = {backwards}; Left = {left}; Right = {right}; Stop = SPACE")
                            active_robots[int(robot_id)].teleop = True
                            state = MenuState.DRIVE
                    except ValueError:
                        pass

                    if not valid:
                        await send_message(websocket, "\r\nInvalid robot ID, try again: ")
                        robot_id = "1"
                        state = MenuState.SELECT

                else:
                    await send_message(websocket, key)
                    robot_id = robot_id + key

            elif state == MenuState.DRIVE:
                id = int(robot_id)
                if key == release:
                    await send_message(websocket, "\r\nReleasing control of robot: " + robot_id)
                    active_robots[id].teleop = False
                    active_robots[id].state = RobotState.STOP
                    state = MenuState.START
                elif key == forwards:
                    await send_message(websocket, "\r\nDriving forwards")
                    active_robots[id].state = RobotState.FORWARDS
                elif key == backwards:
                    await send_message(websocket, "\r\nDriving backwards")
                    active_robots[id].state = RobotState.BACKWARDS
                elif key == left:
                    await send_message(websocket, "\r\nTurning left")
                    active_robots[id].state = RobotState.LEFT
                elif key == right:
                    await send_message(websocket, "\r\nTurning right")
                    active_robots[id].state = RobotState.RIGHT
                elif key == stop:
                    await send_message(websocket, "\r\nStopping")
                    active_robots[id].state = RobotState.STOP
                else:
                    await send_message(websocket, "\r\nUnrecognised command")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()

    loop.run_until_complete(connect_to_server())

    if server_connection is None:
        print(Fore.RED + "[ERROR]: No connection to server")
        sys.exit(1)

    # Specify robots to work with
    # robot_ids = range(2, 3)
    robot_ids = range(1,6)

    for robot_id in robot_ids:
        if robots[robot_id] != '':
            active_robots[robot_id] = Robot(robot_id)
        else:
            print(f"No IP defined for robot {robot_id}")

    print(Fore.GREEN + "[INFO]: Connecting to robots")
    loop.run_until_complete(connect_to_robots())

    if not active_robots:
        print(Fore.RED + "[ERROR]: No connection to robots")
        sys.exit(1)

    # Listen for keyboard input from teleop websocket client
    print(Fore.GREEN + "[INFO]: Starting teleop server")
    # start_server = websockets.serve(ws_handler=handler, host=None, port=7000, ping_interval=None, ping_timeout=None)
    # loop.run_until_complete(start_server)

    # Only communicate with robots that were successfully connected to
    while True:

        print(Fore.GREEN + "[INFO]: Requesting data from server")
        loop.run_until_complete(get_server_data())

        print(Fore.GREEN + "[INFO]: Robots detected:", ids)

        print(Fore.GREEN + "[INFO]: Requesting data from detected robots")
        loop.run_until_complete(get_robot_data(ids))

        print(Fore.GREEN + "Processing...")

        print(Fore.GREEN + "[INFO]: Sending commands to detected robots")
        loop.run_until_complete(send_robot_commands(ids))

        print()

        # TODO: Close websocket connections
        if kill_now():
            loop.run_until_complete(stop_robots(robot_ids)) # Kill all robots, even if not visible
            break

        # Sleep until next control cycle
        time.sleep(0.1)
