
from urx.urscript import URScript
import math
import time
import os

boilerplate =  """
  set_standard_analog_input_domain(0, 1)
  set_standard_analog_input_domain(1, 1)
  set_tool_analog_input_domain(0, 1)
  set_tool_analog_input_domain(1, 1)
  set_analog_outputdomain(0, 0)
  set_analog_outputdomain(1, 0)
  set_tool_voltage(0)
  set_input_actions_to_default()
  set_tcp(p[0.0,0.0,0.0,0.0,0.0,0.0])
  set_payload(0.0)
  set_gravity([0.0, 0.0, 9.82])
  # begin: URCap Installation Node
  #   Source: RG - On Robot, 1.9.0, OnRobot A/S
  #   Type: RG Configuration
  global measure_width=0
  global grip_detected=False
  global lost_grip=False
  global zsysx=0
  global zsysy=0
  global zsysz=0.06935
  global zsysm=0.7415
  global zmasx=0
  global zmasy=0
  global zmasz=0.18659
  global zmasm=0
  global zmasm=0
  global zslax=0
  global zslay=0
  global zslaz=0
  global zslam=0
  global zslam=0
  thread lost_grip_thread():
    while True:
      set_tool_voltage(24)
      if True ==get_digital_in(9):
        sleep(0.024)
        if True == grip_detected:
          if False == get_digital_in(8):
            grip_detected=False
            lost_grip=True
          end
        end
        set_tool_analog_input_domain(0, 1)
        set_tool_analog_input_domain(1, 1)
        zscale = (get_analog_in(2)-0.026)/2.9760034
        zangle = zscale*1.57079633+-0.08726646
        zwidth = 5.0+110*sin(zangle)
        global measure_width = (floor(zwidth*10))/10-9.2
      end
      sync()
    end
  end
  lg_thr = run lost_grip_thread()
  def RG2(target_width=110, target_force=40, payload=0.0, set_payload=False, depth_compensation=False, slave=False):
    grip_detected=False
    if slave:
      slave_grip_detected=False
    else:
      master_grip_detected=False
    end
    timeout = 0
    timeout_limit = 750000
    while get_digital_in(9) == False:
      if timeout > timeout_limit:
        break
      end
      timeout = timeout+1
      sync()
    end
    def bit(input):
      msb=65536
      local i=0
      local output=0
      while i<17:
        set_digital_out(8,True)
        if input>=msb:
          input=input-msb
          set_digital_out(9,False)
        else:
          set_digital_out(9,True)
        end
        if get_digital_in(8):
          out=1
        end
        sync()
        set_digital_out(8,False)
        sync()
        input=input*2
        output=output*2
        i=i+1
      end
      return output
    end
    target_width=target_width+9.2
    if target_force>40:
      target_force=40
    end
    if target_force<4:
      target_force=4
    end
    if target_width>110:
      target_width=110
    end
    if target_width<0:
      target_width=0
    end
    rg_data=floor(target_width)*4
    rg_data=rg_data+floor(target_force/2)*4*111
    rg_data=rg_data+32768
    if slave:
      rg_data=rg_data+16384
    end
    bit(rg_data)
    if depth_compensation:
      finger_length = 55.0/1000
      finger_heigth_disp = 5.0/1000
      center_displacement = 7.5/1000

      start_pose = get_forward_kin()
      set_analog_inputrange(2, 1)
      zscale = (get_analog_in(2)-0.026)/2.9760034
      zangle = zscale*1.57079633+-0.08726646
      zwidth = 5.0+110*sin(zangle)

      start_depth = cos(zangle)*finger_length

      sleep(0.016)
      timeout = 0
      while get_digital_in(9) == True:
        timeout=timeout+1
        sleep(0.008)
        if timeout > 20:
          break
        end
      end
      timeout = 0
      timeout_limit = 750000
      while get_digital_in(9) == False:
        zscale = (get_analog_in(2)-0.026)/2.9760034
        zangle = zscale*1.57079633+-0.08726646
        zwidth = 5.0+110*sin(zangle)
        measure_depth = cos(zangle)*finger_length
        compensation_depth = (measure_depth - start_depth)
        target_pose = pose_trans(start_pose,p[0,0,-compensation_depth,0,0,0])
        if timeout > timeout_limit:
          break
        end
        timeout=timeout+1
        servoj(get_inverse_kin(target_pose),0,0,0.008,0.01,2000)
        if point_dist(target_pose, get_forward_kin()) > 0.005:
          popup("Lower grasping force or max width",title="RG-lag threshold exceeded", warning=False, error=False, blocking=False)
        end
      end
      nspeed = norm(get_actual_tcp_speed())
      while nspeed > 0.001:
        servoj(get_inverse_kin(target_pose),0,0,0.008,0.01,2000)
        nspeed = norm(get_actual_tcp_speed())
      end
        stopj(2)
    end
    if depth_compensation==False:
      timeout = 0
      timeout_count=20*0.008/0.008
      while get_digital_in(9) == True:
        timeout = timeout+1
        sync()
        if timeout > timeout_count:
          break
        end
      end
      timeout = 0
      timeout_limit = 750000
      while get_digital_in(9) == False:
        timeout = timeout+1
        sync()
        if timeout > timeout_limit:
          break
        end
      end
      end
      sleep(0.024)
      if set_payload:
        if slave:
          if get_analog_in(3) < 2:
            zslam=0
          else:
            zslam=payload
          end
        else:
          if get_digital_in(8) == False:
            zmasm=0
          else:
            zmasm=payload
          end
        end
        zload=zmasm+zslam+zsysm
        set_payload(zload,[(zsysx*zsysm+zmasx*zmasm+zslax*zslam)/zload,(zsysy*zsysm+zmasy*zmasm+zslay*zslam)/zload,(zsysz*zsysm+zmasz*zmasm+zslaz*zslam)/zload])
      end
      master_grip_detected=False
      master_lost_grip=False
      slave_grip_detected=False
      slave_lost_grip=False
      if True == get_digital_in(8):
        master_grip_detected=True
      end
      if get_analog_in(3)>2:
        slave_grip_detected=True
      end
      grip_detected=False
      lost_grip=False
      if True == get_digital_in(8):
        grip_detected=True
      end
      zscale = (get_analog_in(2)-0.026)/2.9760034
      zangle = zscale*1.57079633+-0.08726646
      zwidth = 5.0+110*sin(zangle)
      global measure_width = (floor(zwidth*10))/10-9.2
      if slave:
        slave_measure_width=measure_width
      else:
        master_measure_width=measure_width
      end
      return grip_detected
    end
  set_tool_voltage(24)
  set_tcp(p[0,0,0.18659,0,-0,0])
  """
class OnRobotGripperRG2Script(URScript):
    
    def __init__(self):
        super(OnRobotGripperRG2Script, self).__init__()
        # copy the boilerplate to the start of the script to make the RG2() function available
        self.add_line_to_program(boilerplate)

    def _rg2_command(self, target_width=110, target_force=40, payload=0.5, set_payload=False, depth_compensation=False, slave=False):
        self.add_line_to_program("RG2(target_width={}, target_force={}, payload={}, set_payload={}, depth_compensation={}, slave={})"
                                 .format(target_width, target_force, payload, set_payload, depth_compensation, slave))

class OnRobotGripperRG2(object):

    def __init__(self, robot):
        self.robot = robot

    def open_gripper(self, target_width=110, target_force=40, payload=0.5, set_payload=False, depth_compensation=False, slave=False, wait=2):
        urscript = OnRobotGripperRG2Script()
        urscript._rg2_command(target_width, target_force, payload, set_payload, depth_compensation, slave)
        self.robot.send_program(urscript())
        time.sleep(wait)

    def close_gripper(self, target_width=-10, target_force=40, payload=0.5, set_payload=False, depth_compensation=False, slave=False, wait=2):
        urscript = OnRobotGripperRG2Script()
        urscript._rg2_command(target_width, target_force, payload, set_payload, depth_compensation, slave)
        self.robot.send_program(urscript())
        time.sleep(wait)

    @property
    def width(self):
        zscale = (self.robot.get_analog_in(2) - 0.026) / 2.9760034
        zangle = zscale * 1.57079633 + -0.08726646
        zwidth = 5.0 + 110 * math.sin(zangle)
        measure_width = (math.floor(zwidth * 10)) / 10 - 9.2
        return measure_width

    @property
    def object_gripped(self):
        return self.robot.get_digital_in(16) > 0
