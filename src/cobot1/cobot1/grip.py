# set_digital_outputs([1,-2])

import rclpy
import DR_init

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "GripperDA_v1" #워크셀에 등록된 그리퍼 이름

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp # 필요한 기능만 임포트

    # Tool과 TCP 설정
    # set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    # 설정된 설정값 출력
    # print("#"*50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    # print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print("#"*50)


def grip():
    """그리퍼를 닫는 동작 수행"""
    print("그리퍼 닫기")
    from DSR_ROBOT2 import set_digital_outputs, wait_tool_digital_input, ON, tp_log  # 필요한 기능만 임포트

    # 그리퍼 닫기 명령
    set_digital_outputs([1, -2])
    ret = wait_tool_digital_input(index=2, val=ON, timeout=3)
    tp_log("그리퍼 닫기:{}".format(ret == 0))

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('grip')
    DR_init.dsr__node = node

    try:
        initialize_robot()
        grip()
    except KeyboardInterrupt:
        print("프로그램 종료")
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()