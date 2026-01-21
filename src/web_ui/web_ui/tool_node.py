import time

import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import GetCurrentTool, SetCurrentTool

from web_ui.firebase_config import init_firebase, get_reference


class ToolNode(Node):
    def __init__(self):
        super().__init__('tool_node')
        self.get_logger().info("Tool Node 시작")

        init_firebase(self.get_logger())

        # Firebase references
        self.tool_status_ref = get_reference('/robot_status/current_tool')
        self.tool_command_ref = get_reference('/robot_command/set_tool')

        # GetCurrentTool 서비스 클라이언트
        self.get_client = self.create_client(
            GetCurrentTool,
            '/dsr01/tool/get_current_tool'
        )

        # SetCurrentTool 서비스 클라이언트
        # self.set_client = self.create_client(
        #     SetCurrentTool,
        #     '/dsr01/tool/set_current_tool'
        # )

        while not self.get_client.wait_for_service(timeout_sec=1.0):
            pass
        #self.get_logger().info('Waiting for get_current_tool service...')

        # while not self.set_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for set_current_tool service...')

        self.get_logger().info('Tool services ready')

        # 1초마다 현재 Tool 조회
        self.timer = self.create_timer(1.0, self.get_current_tool)

        # Firebase 리스너 설정 (SetCurrentTool 명령 감지)
        #self.tool_command_ref.listen(self.on_set_command)

    def get_current_tool(self):
        """현재 Tool 정보 조회"""
        req = GetCurrentTool.Request()
        future = self.get_client.call_async(req)
        future.add_done_callback(self.get_tool_response_callback)

    def get_tool_response_callback(self, future):
        #self.get_logger().info(f'get Tool Response... {future.result()}')
        try:
            response = future.result()                                                                                                                                                                               
            if response.success:
                self.tool_status_ref.update({
                    'name': response.info,
                    'last_update': time.time()
                })
        except Exception as e:
            self.get_logger().error(f'GetCurrentTool failed: {e}')

    def on_set_command(self, event):
        """Firebase에서 SetCurrentTool 명령 감지"""
        #self.get_logger().info(f'on_set_command--------------------------------------: {event.data}')
        if event.data is None:
            return

        command = event.data
        if command.get('execute') == True:
            tool_name = command.get('name', '')
            self.get_logger().info(f'SetCurrentTool 명령 수신: {tool_name}')
            self.set_current_tool(tool_name)
            self.tool_command_ref.update({'execute': False})

    def set_current_tool(self, tool_name):
        """SetCurrentTool 서비스 호출"""
        req = SetCurrentTool.Request()
        req.name = tool_name

        future = self.set_client.call_async(req)
        
        future.add_done_callback(self.set_tool_response_callback)

    def set_tool_response_callback(self, future):
        self.get_logger().info(f'future--------------------------------------: {future.result()}')
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('SetCurrentTool 성공')
                self.tool_command_ref.update({'status': 'success'})
            else:
                self.get_logger().error('SetCurrentTool 실패')
                self.tool_command_ref.update({'status': 'failed'})
        except Exception as e:
            self.get_logger().error(f'SetCurrentTool failed: {e}')
            self.tool_command_ref.update({'status': f'error: {e}'})


def main(args=None):
    rclpy.init(args=args)
    node = ToolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
