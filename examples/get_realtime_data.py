import urx
import time
import logging

r = urx.Robot("192.168.111.134", use_rt=True, urFirm=5.1)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    while 1:
        try:
            j_temp = r.get_joint_temperature()
            j_voltage = r.get_joint_voltage()
            j_current = r.get_joint_current()
            main_voltage = r.get_main_voltage()
            robot_voltage = r.get_robot_voltage()
            robot_current = r.get_robot_current()

            print("JOINT TEMPERATURE")
            print(j_temp)

            print("JOINT VOLTAGE")
            print(j_voltage)

            print("JOINT CURRENT")
            print(j_current)

            print("MAIN VOLTAGE")
            print(main_voltage)

            print("ROBOT VOLTAGE")
            print(robot_voltage)

            print("ROBOT CURRENT")
            print(robot_current)

            print("##########\t##########\t##########\t##########")
        
            time.sleep(1)

        except:
            pass

    r.close()