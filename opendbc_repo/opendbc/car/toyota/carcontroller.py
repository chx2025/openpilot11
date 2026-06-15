import math
import numpy as np
from opendbc.car import Bus, apply_meas_steer_torque_limits, apply_std_steer_angle_limits, common_fault_avoidance, \
                        make_tester_present_msg, rate_limit, structs, ACCELERATION_DUE_TO_GRAVITY, DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.common.pid import PIDController
from opendbc.car.secoc import add_mac, build_sync_mac
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.toyota import toyotacan
from opendbc.car.toyota.values import CAR, STATIC_DSU_MSGS, NO_STOP_TIMER_CAR, TSS2_CAR, \
                                        CarControllerParams, ToyotaFlags, \
                                        UNSUPPORTED_DSU_CAR
from opendbc.can import CANPacker
from openpilot.common.params import Params

Ecu = structs.CarParams.Ecu
LongCtrlState = structs.CarControl.Actuators.LongControlState
SteerControlType = structs.CarParams.SteerControlType
VisualAlert = structs.CarControl.HUDControl.VisualAlert

ACCEL_WINDUP_LIMIT = 4.0 * DT_CTRL * 3
ACCEL_WINDDOWN_LIMIT = -4.0 * DT_CTRL * 3
ACCEL_PID_UNWIND = 0.03 * DT_CTRL * 3

MAX_STEER_RATE = 100
MAX_STEER_RATE_FRAMES = 18
MAX_USER_TORQUE = 500

def get_long_tune(CP, params):
  if CP.carFingerprint in TSS2_CAR:
    kiBP = [2., 5.]
    kiV = [0.5, 0.25]
  else:
    kiBP = [0., 5., 35.]
    kiV = [3.6, 2.4, 1.5]
  return PIDController(0.0, (kiBP, kiV), k_f=1.0,
                       pos_limit=params.ACCEL_MAX, neg_limit=params.ACCEL_MIN,
                       rate=1 / (DT_CTRL * 3))

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(self.CP)
    self.last_torque = 0
    self.last_angle = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.permit_braking = True
    self.steer_rate_counter = 0
    self.distance_button = 0
    self.long_pid = get_long_tune(self.CP, self.params)
    self.aego = FirstOrderFilter(0.0, 0.25, DT_CTRL * 3)
    self.pitch = FirstOrderFilter(0, 0.5, DT_CTRL)
    self.accel = 0
    self.prev_accel = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.secoc_lka_message_counter = 0
    self.secoc_lta_message_counter = 0
    self.secoc_prev_reset_counter = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    stopping = actuators.longControlState == LongCtrlState.stopping
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    if len(CC.orientationNED) == 3:
      self.pitch.update(CC.orientationNED[1])

    can_sends = []
    params = Params()
    steerMax = params.get_int("CustomSteerMax")
    steerDeltaUp = params.get_int("CustomSteerDeltaUp")
    steerDeltaDown = params.get_int("CustomSteerDeltaDown")
    self.params.STEER_MAX = self.params.STEER_MAX if steerMax <= 0 else steerMax
    self.params.STEER_DELTA_UP = self.params.STEER_DELTA_UP if steerDeltaUp <= 0 else steerDeltaUp
    self.params.STEER_DELTA_DOWN = self.params.STEER_DELTA_DOWN if steerDeltaDown <= 0 else steerDeltaDown

    # ====================== 优化版·卡罗拉专用 ======================
    v = CS.out.vEgo
    speed_list = [0, 15, 30, 50, 110]
    gain_list  = [1.15, 1.12, 1.08, 1.05, 1.03]
    steer_scale = float(np.interp(v, speed_list, gain_list))

    angle_err = abs(actuators.steeringAngleDeg - CS.out.steeringAngleDeg)
    if angle_err > 8.0:
        steer_scale *= 1.08

    steer_desired_deg = actuators.steeringAngleDeg * 1.18

    right_turn_comp = 0.0
    if steer_desired_deg > 4.0:
        right_turn_comp = 2.5
    steer_desired_deg += right_turn_comp

    new_torque = int(round(actuators.torque * self.params.STEER_MAX *         steer_scale))
    # =================================================================

    apply_torque = apply_meas_steer_torque_limits(new_torque, self.last_torque, CS.out.steeringTorqueEps, self.params)
    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, lat_active,
                                                                      self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

    if not lat_active:
      apply_torque = 0

    if self.CP.steerControlType == SteerControlType.angle:
      apply_torque = 0
      apply_steer_req = False
      if self.frame % 2 == 0:
        apply_angle = steer_desired_deg + CS.out.steeringAngleOffsetDeg
        self.last_angle = apply_std_steer_angle_limits(apply_angle, self.last_angle, CS.out.vEgoRaw,
                                                       CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg,
                                                       CC.latActive, self.params.ANGLE_LIMITS)

    self.last_torque = apply_torque
    steer_command = toyotacan.create_steer_command(self.packer, apply_torque, apply_steer_req)

    if self.CP.flags & ToyotaFlags.SECOC.value:
      steer_command = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_lka_message_counter,
                              steer_command)
      self.secoc_lka_message_counter += 1
    can_sends.append(steer_command)

    if self.frame % 2 == 0 and self.CP.carFingerprint in TSS2_CAR:
      lta_active = lat_active and self.CP.steerControlType == SteerControlType.angle
      full_torque_condition = (abs(CS.out.steeringTorqueEps) < self.params.STEER_MAX and
                               abs(CS.out.steeringTorque) < self.params.MAX_LTA_DRIVER_TORQUE_ALLOWANCE)
      torque_wind_down = 100 if lta_active and full_torque_condition else 0
      can_sends.append(toyotacan.create_lta_steer_command(self.packer, self.CP.steerControlType, self.last_angle,
                                                          lta_active, self.frame // 2, torque_wind_down))

      if self.CP.flags & ToyotaFlags.SECOC.value:
        lta_steer_2 = toyotacan.create_lta_steer_command_2(self.packer, self.frame // 2)
        lta_steer_2 = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_lta_message_counter,
                              lta_steer_2)
        self.secoc_lta_message_counter += 1
        can_sends.append(lta_steer_2)

    if CS.out.standstill and not self.last_standstill and (self.CP.carFingerprint not in NO_STOP_TIMER_CAR):
      self.standstill_req = True
    if CS.pcm_acc_status != 8:
      self.standstill_req = False
    self.last_standstill = CS.out.standstill

    fcw_alert = hud_control.visualAlert == VisualAlert.fcw
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    lead = hud_control.leadVisible or CS.out.vEgo < 12.

    if self.CP.openpilotLongitudinalControl:
      if self.frame % 3 == 0:
        if self.frame % 6 == 0 and self.CP.openpilotLongitudinalControl:
          desired_distance = 4 - hud_control.leadDistanceBars
          if CS.out.cruiseState.enabled and CS.pcm_follow_distance != desired_distance:
            self.distance_button = not self.distance_button
          else:
            self.distance_button = 0

        pcm_accel_cmd = actuators.accel
        if CC.longActive:
          pcm_accel_cmd = rate_limit(pcm_accel_cmd, self.prev_accel, ACCEL_WINDDOWN_LIMIT, ACCEL_WINDUP_LIMIT)
        self.prev_accel = pcm_accel_cmd

        accel_due_to_pitch = math.sin(min(self.pitch.x, 0.0)) * ACCELERATION_DUE_TO_GRAVITY
        net_acceleration_request = pcm_accel_cmd + accel_due_to_pitch

        if not self.CP.flags & ToyotaFlags.SECOC.value:
          a_ego_blended = float(np.interp(CS.out.vEgo, [1.0, 2.0], [CS.gvc, CS.out.aEgo]))
        else:
          a_ego_blended = CS.out.aEgo

        prev_aego = self.aego.x
        self.aego.update(a_ego_blended)
        j_ego = (self.aego.x - prev_aego) / (DT_CTRL * 3)
        future_t = float(np.interp(CS.out.vEgo, [2., 5.], [0.25, 0.5]))
        a_ego_future = a_ego_blended + j_ego * future_t

        if CC.longActive:
          self.long_pid.i -= ACCEL_PID_UNWIND * float(np.sign(self.long_pid.i))
          error_future = pcm_accel_cmd - a_ego_future
          pcm_accel_cmd = self.long_pid.update(error_future,
                                               speed=CS.out.vEgo,
                                               feedforward=pcm_accel_cmd,
                                               freeze_integrator=actuators.longControlState != LongCtrlState.pid)
        else:
          self.long_pid.reset()

        net_acceleration_request_min = min(actuators.accel + accel_due_to_pitch, net_acceleration_request)
        if net_acceleration_request_min < 0.0 or stopping or not CC.longActive:
            self.permit_braking = True
        elif net_acceleration_request_min > 0.0:
            self.permit_braking = False

        if accel_due_to_pitch < -0.3:
                if -0.15 < pcm_accel_cmd < 0.15:
                        pcm_accel_cmd = 0.0
        else:
                if -0.05 < pcm_accel_cmd < 0.05:
                        pcm_accel_cmd = 0.0

        pcm_accel_cmd = float(np.clip(pcm_accel_cmd, self.params.ACCEL_MIN, self.params.ACCEL_MAX))
        can_sends.append(toyotacan.create_accel_command(self.packer,
        pcm_accel_cmd, pcm_cancel_cmd, self.permit_braking,
        self.standstill_req, lead, CS.acc_type, fcw_alert, self.distance_button))
        self.accel = pcm_accel_cmd
    else:
      if pcm_cancel_cmd:
        if self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
          can_sends.append(toyotacan.create_acc_cancel_command(self.packer))
        else:
          can_sends.append(toyotacan.create_accel_command(self.packer, 0, pcm_cancel_cmd, True, False, lead, CS.acc_type, False, self.distance_button))

    if self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      send_ui = False
      if ((fcw_alert or steer_alert) and not self.alert_active) or (not (fcw_alert or steer_alert) and self.alert_active):
        send_ui = True
        self.alert_active = not self.alert_active
      elif pcm_cancel_cmd:
        send_ui = True

      if self.frame % 20 == 0 or send_ui:
        can_sends.append(toyotacan.create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, hud_control.leftLaneVisible,
                                                     hud_control.rightLaneVisible, hud_control.leftLaneDepart,
                                                     hud_control.rightLaneDepart, CC.enabled, CS.lkas_hud))

      if (self.frame % 100 == 0 or send_ui) and (self.CP.enableDsu or self.CP.flags & ToyotaFlags.DISABLE_RADAR.value):
        can_sends.append(toyotacan.create_fcw_command(self.packer, fcw_alert))

    for addr, cars, bus, fr_step, vl in STATIC_DSU_MSGS:
      if self.frame % fr_step == 0 and self.CP.enableDsu and self.CP.carFingerprint in cars:
        can_sends.append(CanData(addr, vl, bus))

    if self.frame % 20 == 0 and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      can_sends.append(make_tester_present_msg(0x750, 0, 0xF))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    new_actuators.steeringAngleDeg = self.last_angle
    new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends
