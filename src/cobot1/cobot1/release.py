def main():
    set_digital_outputs([-1,2])
    ret = wait_tool_digital_input(index=2, val=ON, timeout=3)
    tp_log("그리퍼 열기:{}".format(ret==0))


if __name__ == "__main__":
    main()