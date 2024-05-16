import time
from rclpy.node import Node
import rclpy

# from pros_car_py.car_models import *
# from rclpy.duration import Duration

# import orjson
from std_msgs.msg import String
import curses
import threading
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

# from pros_car_py.env import *
import math
import serial


class TankKeyboardController(Node):
    def __init__(self, stdscr, vel: float = 300):
        super().__init__("car_C_keyboard")
        self.vel = vel
        self.rotate_speed = vel
        self.ESP32 = serial.Serial("/dev/ttyACM0", 115200, timeout=0.5)

        self.stdscr = stdscr
        curses.noecho()
        curses.raw()
        self.stdscr.keypad(False)
        self.key_in_count = 0
        self._car_state_msg = ""

        self._left_tire = 0  # rad/s
        self._right_tire = 0

    def run(self, vel=None):
        if vel is None:
            vel = self.vel
        self.stdscr.nodelay(True)
        try:
            while rclpy.ok():
                c = self.stdscr.getch()

                # Check if a key was actually pressed
                if c != curses.ERR:
                    self.key_in_count += 1
                    # self.print_basic_info(c)
                    if c == ord("w"):
                        self.handle_key_w(vel)
                    elif c == ord("a"):
                        self.handle_key_a(vel)
                    elif c == ord("s"):
                        self.handle_key_s(vel)
                    elif c == ord("d"):
                        self.handle_key_d(vel)
                    elif c == ord("e"):
                        self.handle_key_e(vel)
                    elif c == ord("r"):
                        self.handle_key_r(vel)
                    elif c == ord("z"):
                        self.handle_key_z()
                    elif c == ord("q"):
                        break
                    print()
                else:
                    # self.print_basic_info(ord(" "))
                    time.sleep(0.01)

        finally:
            curses.endwin()

    def handle_key_w(self, vel: float = 300):
        # Your action for the 'w' key here
        self.stdscr.addstr(f"car go forward")
        self._left_tire = vel  # rad/s
        self._right_tire = vel  # rad/s
        data = {}
        data["RS"] = self._right_tire
        data["LS"] = self._left_tire
        self.ESP32.write((str(data) + "\r\n").encode())
        pass

    def handle_key_a(self, vel: float = 300, rate: float = 5):
        # Your action for the 'a' key here
        self.stdscr.addstr(f"car go left")
        self._left_tire = vel / rate  # rad/s
        self._right_tire = vel  # rad/s
        data = {}
        data["RS"] = self._right_tire
        data["LS"] = self._left_tire
        self.ESP32.write((str(data) + "\r\n").encode())

        pass

    # Add methods for other keys similarly
    def handle_key_s(self, vel: float = 300):
        self.stdscr.addstr(f"car go backward")
        self._left_tire = -vel  # rad/s
        self._right_tire = -vel  # rad/s

        data = {}
        data["RS"] = self._right_tire
        data["LS"] = self._left_tire
        self.ESP32.write((str(data) + "\r\n").encode())

        pass

    def handle_key_d(self, vel: float = 300, rate: float = 5):
        self.stdscr.addstr(f"car go right")
        self._left_tire = vel  # rad/s
        self._right_tire = vel / rate  # rad/s

        data = {}
        data["RS"] = self._right_tire
        data["LS"] = self._left_tire
        self.ESP32.write((str(data) + "\r\n").encode())

        pass

    def handle_key_e(self, vel: float = 300):
        self.stdscr.addstr(f"car go clockwise")
        self._left_tire = vel  # rad/s
        self._right_tire = -vel  # rad/s

        data = {}
        data["RS"] = self._right_tire
        data["LS"] = self._left_tire
        self.ESP32.write((str(data) + "\r\n").encode())

        pass

    def handle_key_r(self, vel: float = 300):
        self.stdscr.addstr(f"car go counterclockwise")
        self._left_tire = -vel  # rad/s
        self._right_tire = vel  # rad/s

        data = {}
        data["RS"] = self._right_tire
        data["LS"] = self._left_tire
        self.ESP32.write((str(data) + "\r\n").encode())

        pass

    def handle_key_z(self):
        self.stdscr.addstr(f"stop car")
        self._left_tire = 0  # rad/s
        self._right_tire = 0

        data = {}
        data["RS"] = self._right_tire
        data["LS"] = self._left_tire
        self.ESP32.write((str(data) + "\r\n").encode())

        pass


# ... Rest of your code, e.g. initializing rclpy and running the node
def main(args=None):
    rclpy.init(args=args)
    stdscr = curses.initscr()
    vel = 300
    node = TankKeyboardController(stdscr, vel=vel)

    # Spin the node in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        curses.endwin()
        node.get_logger().info(f"Quit keyboard!")
        rclpy.shutdown()
        spin_thread.join()  # Ensure the spin thread is cleanly stopped


if __name__ == "__main__":
    main()
