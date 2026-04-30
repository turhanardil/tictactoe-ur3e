"""
Game AI node for the tictactoe-ur3e capstone.

Subscribes to /board_state, runs minimax to decide the robot's next
move, and publishes /move_command. Robot is player 2 (blue tokens),
human is player 1 (red tokens), 0 is empty. Human plays first.

Minimax is exhaustive (the game tree has only 9! = 362880 leaves and
branching is bounded by 9 down to 1). The robot plays optimally and
will never lose.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from tictactoe_msgs.msg import BoardState, MoveCommand


LINES = (
    (0, 1, 2), (3, 4, 5), (6, 7, 8),
    (0, 3, 6), (1, 4, 7), (2, 5, 8),
    (0, 4, 8), (2, 4, 6),
)


def winner(board):
    for a, b, c in LINES:
        if board[a] != 0 and board[a] == board[b] == board[c]:
            return board[a]
    return 0


def minimax(board, ai, current):
    w = winner(board)
    if w == ai:
        return 1, -1
    if w != 0:
        return -1, -1
    if 0 not in board:
        return 0, -1
    is_max = current == ai
    best_score = -2 if is_max else 2
    best_cell = -1
    for i in range(9):
        if board[i] != 0:
            continue
        board[i] = current
        score, _ = minimax(board, ai, 3 - current)
        board[i] = 0
        if is_max and score > best_score:
            best_score, best_cell = score, i
        elif not is_max and score < best_score:
            best_score, best_cell = score, i
    return best_score, best_cell


class GameAI(Node):

    def __init__(self):
        super().__init__('game_ai')
        latching = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._sub = self.create_subscription(
            BoardState, '/board_state', self._on_board, latching)
        self._pub = self.create_publisher(MoveCommand, '/move_command', 10)
        self.ai = 2
        self.human = 1
        self._last_acted_state = None
        self.get_logger().info('game_ai ready, waiting for /board_state')

    def _on_board(self, msg):
        board = list(msg.cells)
        state_key = tuple(board)
        if state_key == self._last_acted_state:
            return
        w = winner(board)
        if w == self.ai:
            self.get_logger().info('robot wins, no further moves')
            return
        if w == self.human:
            self.get_logger().error(
                'human wins (minimax should make this unreachable)')
            return
        if 0 not in board:
            self.get_logger().info('draw, no further moves')
            return
        n_h = sum(1 for c in board if c == self.human)
        n_a = sum(1 for c in board if c == self.ai)
        if n_h <= n_a:
            return  # human plays first, so AI only acts when n_h > n_a
        score, cell = minimax(board, self.ai, self.ai)
        if cell == -1:
            self.get_logger().warn('no available cell')
            return
        cmd = MoveCommand()
        cmd.cell_index = cell
        self._pub.publish(cmd)
        self._last_acted_state = state_key
        self.get_logger().info(
            'AI plays cell %d (minimax score=%d)' % (cell, score))


def main():
    rclpy.init()
    node = GameAI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
