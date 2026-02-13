import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

# PyQt6 구성 요소
from PyQt6.QtWidgets import QApplication, QWidget, QGridLayout, QPushButton, QVBoxLayout, QLabel
from PyQt6.QtCore import QThread

class RosSpinThread(QThread):
    def run(self):
        rclpy.spin(gui_node)

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.reset_srv = self.create_client(Empty, '/reset_simulation')

    # topic : 실시간 이동 제어 (단방향 전송)
    def move_robot(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)

    # service : 특정 이벤트 담당(예시: 시뮬레이션 초기화) (양방향)
    def call_reset(self):
        if not self.reset_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('error search Service')
            return
        request = Empty.Request()
        self.reset_srv.call_async(request)
        self.get_logger().info('Service')

    # action : 시간이 걸리는 복잡한 미션 -> 자율주행에 적합
    def call_action(self):
        self.get_logger().info("Action")
        self.move_robot(0.0, 1.0)

# GUI logic
class RobotGui(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout()
        grid = QGridLayout()

        # topic logic
        main_layout.addWidget(QLabel("Topic"))
        main_layout.addLayout(grid)

        btn_up = QPushButton("▲")
        btn_down = QPushButton("▼")
        btn_left = QPushButton("◀")
        btn_right = QPushButton("▶")
        btn_stop = QPushButton("S")

        grid.addWidget(btn_up, 0, 1)
        grid.addWidget(btn_left, 1, 0)
        grid.addWidget(btn_stop, 1, 1)
        grid.addWidget(btn_right, 1, 2)
        grid.addWidget(btn_down, 2, 1)

        btn_up.clicked.connect(lambda: gui_node.move_robot(0.2, 0.0))
        btn_down.clicked.connect(lambda: gui_node.move_robot(-0.2, 0.0))
        btn_left.clicked.connect(lambda: gui_node.move_robot(0.0, 0.5))
        btn_right.clicked.connect(lambda: gui_node.move_robot(0.0, -0.5))
        btn_stop.clicked.connect(lambda: gui_node.move_robot(0.0, 0.0))

        # service logic
        main_layout.addWidget(QLabel("\nService"))
        btn_reset = QPushButton("Reset")
        btn_reset.clicked.connect(gui_node.call_reset)
        main_layout.addWidget(btn_reset)

        # action logic
        main_layout.addWidget(QLabel("\nAction"))
        btn_action = QPushButton("Start Action")
        btn_action.clicked.connect(gui_node.call_action)
        main_layout.addWidget(btn_action)

        self.setLayout(main_layout)
        self.setWindowTitle('turtlebot_control')
        self.setFixedSize(250, 350)
        self.show()

# main
def main(args=None):
    global gui_node
    rclpy.init(args=args)

    gui_node = TurtleBotController()

    spin_thread = RosSpinThread()
    spin_thread.start()

    app = QApplication(sys.argv)
    window = RobotGui()

    sys.exit(app.exec())

if __name__ == '__main__':
    main()


