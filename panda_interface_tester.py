#!/usr/bin/env python3
import time

import zmq

import selfdrive.messaging as messaging
from selfdrive.car.hyundai.carstate import CarState, get_can_parser, get_camera_parser
from selfdrive.controls.lib.vehicle_model import VehicleModel
from common.params import Params
from selfdrive.car.hyundai.interface import CarInterface
from selfdrive.car.hyundai.values import CAR, DBC
from selfdrive.services import service_list
from selfdrive.config import Conversions as CV
from selfdrive.car.vin import get_vin, VIN_UNKNOWN


class CorollaInterface:
    def __init__(self):
        self.car_params = CarInterface.get_params(CAR.ELANTRA)

        self.can_poller = zmq.Poller()
        self.can_sock = messaging.sub_sock(service_list["can"].port)
        self.can_poller.register(self.can_sock)

        self.car_state = CarState(self.car_params)
        self.can_parser = get_can_parser(self.car_params)
        self.can_cam_parser = get_camera_parser(self.car_params)
        self.vehicle_model = VehicleModel(self.car_params)

        # Set CarVin because boardd checks for CarVin in params before
        # setting the safety model; otherwise, the safety thread will wait for it
        Params().put("CarVin", VIN_UNKNOWN)
        Params().put("CarParams", self.car_params.to_bytes())

        dbc_name = DBC[self.car_params.carFingerprint]["pt"]
        # self.messenger = CorollaMessenger(dbc_name, self.car_params)
        self.frame = 0

        self.control_data = None
        self.sendcan = messaging.pub_sock(service_list["sendcan"].port)

    def read_car_state(self):
        #print("inside read_car_state")
        can_strings = messaging.drain_sock_raw_poller(
            self.can_poller, self.can_sock, wait_for_one=False
        )
        #print("gonna update strings")
        self.can_parser.update_strings(can_strings)
        print("\n\n\ngonna update car state")
        self.car_state.update(self.can_parser, self.can_cam_parser)
        #print("adjusting yw rate")
        self.car_state.yaw_rate = self.vehicle_model.yaw_rate(
            self.car_state.angle_steers * CV.DEG_TO_RAD, self.car_state.v_ego
        )

        return self.car_state

obd_msg_attr = ['vEgo', 'gas', 'gasPressed', 'brake', 'brakePressed', 'steeringAngle','steeringTorque', 'steeringPressed', 'gearShifter', 'steeringRate', 'aEgo', 'vEgoRaw', 'standstill', 'brakeLights', 'leftBlinker', 'rightBlinker', 'yawRate', 'genericToggle', 'doorOpen', 'seatbeltUnlatched', 'canValid', 'steeringTorqueEps', 'clutchPressed']

if __name__ == '__main__':
  CS = CorollaInterface()
  print("initialized Interface")
  i = 0;
  while 1:
    i += 1;
    print("gonna read state")
    new_car_state = CS.read_car_state()
    print("iteration number ", i)
    #for info in obd_msg_attr:
    print("\nvEgo ", new_car_state.v_ego)
    print("\naEgo  ", new_car_state.a_ego)
    print("\nLeft Blinker ON  ", new_car_state.left_blinker_on)
    print("\nRight Blinker ON  ", new_car_state.right_blinker_on)
    print("\nleft_blinker_flash  ", new_car_state.left_blinker_flash)
    print("\nright_blinker_flash  ", new_car_state.right_blinker_flash)
    print("\nbrake_pressed  ", new_car_state.brake_pressed)
    print("\npark_brake  ", new_car_state.park_brake)
    print("\nGear Shifter ", new_car_state.gear_shifter)
    print("\nGear Shifter Cluster  ", new_car_state.gear_shifter_cluster)
    
    time.sleep(0.01)
