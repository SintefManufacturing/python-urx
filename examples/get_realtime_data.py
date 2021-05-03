import urx
import time
import logging


if __name__ == "__main__":
    while 1:
        try:
            r = urx.Robot("192.168.137.129", use_rt=True, urFirm=5.1)
            break
        except:
            pass

    logging.basicConfig(level=logging.INFO)
    while 1:
        try:
            j_temp = r.get_joint_temperature()
            j_voltage = r.get_joint_voltage()
            j_current = r.get_joint_current()
            main_voltage = r.get_main_voltage()
            robot_voltage = r.get_robot_voltage()
            robot_current = r.get_robot_current()
            elbow_position = r.get_elbow_position()
            elbow_velocity = r.get_elbow_velocity()

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

            print("ELBOW POSITION")
            print(elbow_position)

            print("ELBOW VELOCITY")
            print(elbow_velocity)
            
            print("##########\t##########\t##########\t##########")

        except:
            pass
        
        time.sleep(1)

    r.close()