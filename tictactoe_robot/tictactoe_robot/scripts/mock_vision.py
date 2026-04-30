"""
Mock vision node for the tictactoe-ur3e capstone.

Replaces a real camera. Reads commands from stdin and maintains
the authoritative game board state, publishing it as a latched
BoardState message every time the state changes.

Commands:
  r0 .. r8    Human places a red token at that cell index.
  restart     Reset the board to all empty.
  quit        Exit the node cleanly.

Subscribes to /move_command so that when the game_ai (and later the
arm_control node) decides on a robot move, mock_vision applies it to
the same authoritative state.
"""

import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from tictactoe_msgs.msg import BoardState, MoveCommand


LINES = (
    (0, 1, 2), (3, 4, 5), (6, 7, 8),
    (0, 3, 6), (1, 4, 7), (2, 5, 8),
    (0, 4, 8), (2, 4, 6),
)


def render(cells):
    sym = {0: '.', 1: 'H', 2: 'R'}
    rows = [' '.join(sym[c] for c in cells[i:i + 3]) for i in (0, 3, 6)]
    return '\n  '.join(rows)


def is_game_over(cells):
    for a, b, c in LINES:
        if cells[a] != 0 and cells[a] == cells[b] == cells[c]:
            return True
    return 0 not in cells


class MockVision(Node):

    def __init__(self):
        super().__init__('mock_vision')
        latching = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(BoardState, '/board_state', latching)
        self._sub = self.create_subscription(
            MoveCommand, '/move_command', self._on_move_command, 10)
        self._lock = threading.Lock()
        self.cells = [0] * 9
        self.awaiting_ai = False
        self._publish()

    def _publish(self):
        msg = BoardState()
        msg.cells = list(self.cells)
        self._pub.publish(msg)
        self.get_logger().info(
            'board_state %s\n  %s' % (self.cells, render(self.cells)))

    def _on_move_command(self, msg):
        with self._lock:
            idx = msg.cell_index
            # Always release the wait flag on any /move_command, even if rejected.
            # Keeps the user unblocked when the AI publishes garbage.
            self.awaiting_ai = False
            if not 0 <= idx <= 8:
                self.get_logger().warn('invalid /move_command idx=%d' % idx)
                return
            if self.cells[idx] != 0:
                self.get_logger().warn(
                    'AI tried occupied cell %d (current=%d)'
                    % (idx, self.cells[idx]))
                return
            self.cells[idx] = 2
            self.get_logger().info('AI played cell %d' % idx)
            self._publish()

    def handle_line(self, line):
        with self._lock:
            line = line.strip().lower()
            if not line:
                return True
            if line in ('quit', 'exit'):
                return False
            if line == 'restart':
                self.cells = [0] * 9
                self.awaiting_ai = False
                self.get_logger().info('board reset')
                self._publish()
                return True
            if line.startswith('r'):
                if self.awaiting_ai:
                    self.get_logger().warn(
                        'waiting for AI move, ignored "%s"' % line)
                    return True
                try:
                    idx = int(line[1:])
                except ValueError:
                    self.get_logger().warn(
                        'bad input "%s", use r0..r8' % line)
                    return True
                if not 0 <= idx <= 8:
                    self.get_logger().warn(
                        'cell %d out of range' % idx)
                    return True
                if self.cells[idx] != 0:
                    self.get_logger().warn(
                        'cell %d taken (=%d)' % (idx, self.cells[idx]))
                    return True
                self.cells[idx] = 1
                self.get_logger().info('human played cell %d' % idx)
                self._publish()
                if not is_game_over(self.cells):
                    self.awaiting_ai = True
                return True
            self.get_logger().warn(
                'unknown "%s" (use r0..r8, restart, quit)' % line)
            return True


def main():
    rclpy.init()
    node = MockVision()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print('mock_vision ready. r0..r8 to play, restart, quit.', flush=True)
    try:
        while True:
            try:
                line = input('mv> ')
            except EOFError:
                break
            if not node.handle_line(line):
                break
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
