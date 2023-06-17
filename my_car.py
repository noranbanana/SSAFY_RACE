from DrivingInterface.drive_controller import DrivingController
import math

class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = False

        # api or keyboard
        self.enable_api_control = True # True(Controlled by code) /False(Controlled by keyboard)
        super().set_enable_api_control(self.enable_api_control)

        #
        # Editing area ends
        # ==========================================================#
        super().__init__()

    def tr_steer_count(self, car_controls, sensing_info):

        ft_angs = sensing_info.track_forward_angles
        ft_ang = ft_angs[0]
        t_val_plus = ft_angs[2] - ft_angs[1]
        t_plus = 50
        c_st = ft_ang / 90
        co_th_dscp = 7 / 9
        co_br_ascp = 1 / 9

        if 1 / 12 < c_st < 2 / 3:
            if ft_ang >= 0:
                ft_ang += 16
            elif ft_ang < 0:
                ft_ang -= 16

            if ft_ang >= 90:
                ft_ang = 90
            elif ft_ang < -90:
                ft_ang = -90

            c_st = (ft_ang) / 90
        # if t_val_plus > 0:
        #     c_di_ang = t_val_plus + 40
        # elif t_val_plus < 0:
        #     c_di_ang = t_val_plus - 40

        car_controls.steering = c_st;
        car_controls.throttle = 1 - abs(c_st) * co_th_dscp;

    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #

        if self.is_debug:
            print("=========================================================")
            print("[MyCar] to middle: {}".format(sensing_info.to_middle))

            print("[MyCar] collided: {}".format(sensing_info.collided))
            print("[MyCar] car speed: {} km/h".format(sensing_info.speed))

            print("[MyCar] is moving forward: {}".format(sensing_info.moving_forward))
            print("[MyCar] moving angle: {}".format(sensing_info.moving_angle))
            print("[MyCar] lap_progress: {}".format(sensing_info.lap_progress))

            print("[MyCar] track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("[MyCar] track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            print("[MyCar] opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            print("[MyCar] distance_to_way_points: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        ###########################################################################

        # 초기 환경
        # init
        car_controls.steering = 0
        car_controls.throttle = 0.8
        car_controls.brake = 0

        # 기본 자동차 회전각(ct_ang) 사용
        # 현재 자동차와 트랙사이의 각도를 회전각 조정의 기본으로 사용한다.
        # 현재 차와 트랙사이 각도
        ct_ang = sensing_info.moving_angle

        # 트랙이 휘어지는 경우 회전각 보정
        # 트랙 사이 각도 변화를 고려한다.
        # 이때 10m앞 트랙과 20m앞 트랙사이의 각도(ft_tr12_ang)를 고려하여 기본 회전각을 보정한다.
        # - 효과1: ct_ang은 현재 자동차 위치를 기준으로 하므로 앞방향에서 휘어지는 경우를 고려하기 힘들다.
        # - 고로 ft_tr12_ang를 고려해 선제적으로 회전각을 더한다.
        # - 효과2: 트랙이 휘어지는 크기와 ft_tr12_ang값은 비례한다. -> 기본 회전 보정

        # ft_ang1: 10m앞 트랙(1)과 자동차 사이 각도
        # ft_ang2: 20m앞 트랙(2)과 자동차 사이 각도
        # ft_dist1: 10m앞 트랙(1)과 자동차 사이 거리
        # ft_dist2: 20m앞 트랙(2)과 자동차 사이 거리
        # ft_tr12_ang: 트랙1, 트랙2사이의 각도

        # ft_tr12_ang를 구하기 위해 현재 위치를 기준으로 위 값들과 tan, artan를 사용한다.
        # 현재위치에서 트랙1과 트랙2위치를 삼각형 기준으로 보고 사이 각도를 구한다.
        ft_angs = sensing_info.track_forward_angles
        ft_dists = sensing_info.distance_to_way_points

        ft_ang1 = ft_angs[0]
        ft_ang2 = ft_angs[1]

        ft_dist1 = ft_dists[0]
        ft_dist2 = ft_dists[1]

        td_h = ft_dist2 - ft_dist1
        # 라디안 고려
        td_w = round(math.tan(math.radians(ft_ang2))*ft_dist2,2) - round(math.tan(math.radians(ft_ang1))*ft_dist1,2)
        ft_tr12_ang = round(math.degrees(round(math.atan(td_w/td_h),2)),2)

        # 기본 자동차 회전 조정 비율
        max_ct_ang = 60
        
        # 코너를 돌경우의 엑셀, 브레이크 비율
        co_th_dscp = 2 / 3
        co_br_ascp = 2 / 9
        ft_tr_dscp = 1 / 3

        # 회전각도: 0 - 3 (미세)
        # 회전각도: 3 - 7.2 (중간)
        # 중간각 회전 보정
        if  ft_ang1 != 0 and 4.8 <= abs(ct_ang) < 7.2:
            if ct_ang >= 0:
                ct_ang += 6
            elif ct_ang < 0:
                ct_ang -= 6
        elif ft_ang1 != 0 and 3.2 <= abs(ct_ang) < 4.8:
            if ct_ang >= 0:
                ct_ang += 6
            elif ct_ang < 0:
                ct_ang -= 6

        # 회전각 max 초과 보정
        if ct_ang >= max_ct_ang:
            ct_ang = max_ct_ang
        elif ct_ang < -max_ct_ang:
            ct_ang = -max_ct_ang
        
        # 기본 자동차 회전 값
        c_st = -ct_ang / max_ct_ang

        # 전방의 트랙사이 각도(tr12_ang) 조정 비율
        max_tr12_ang = 90
        c_tr12_st = 0
        c_tr12_st = ft_tr12_ang/90
        
        # 기본 자동차 회전 + 전방 트랙사이 각도
        if 6 < abs(ft_tr12_ang):
            c_st += c_tr12_st

        car_controls.steering = c_st

        # 코너 돌시 속도 보정
        if 2 <= abs(ct_ang):
            car_controls.throttle = 1 - abs(c_st*co_th_dscp)
            if sensing_info.speed > 80 and abs(c_st) > 1/2:
                car_controls.brake = abs(c_st)*co_br_ascp;

        print("현재 차 각도: ", sensing_info.moving_angle)
        print("전방 트랙사이 각도: ", ft_tr12_ang)
        print("c_st: ", car_controls.steering)
        print("c_th: ", car_controls.throttle)
        print("c_br: ", car_controls.brake)
        print()

        if self.is_debug:
            print("[MyCar] steering:{}, throttle:{}, brake:{}"
                  .format(car_controls.steering, car_controls.throttle, car_controls.brake))

        #
        # Editing area ends
        # ==========================================================#
        return car_controls


    # ============================
    # If you have NOT changed the <settings.json> file
    # ===> player_name = ""
    #
    # If you changed the <settings.json> file
    # ===> player_name = "My car name" (specified in the json file)  ex) Car1
    # ============================
    def set_player_name(self):
        player_name = ""
        return player_name


if __name__ == '__main__':
    print("[MyCar] Start Bot! (PYTHON)")

    client = DrivingClient()
    return_code = client.run()

    print("[MyCar] End Bot! (PYTHON)")

    exit(return_code)
