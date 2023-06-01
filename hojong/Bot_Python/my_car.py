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

    def merge_direction(self, sensing_info):
        direction = [0,0,0,0,0] #forward len, angle_sum, len, angle_sum2, len2
        second = False
        angle_sum = 0

        
        for i in range(0,len(sensing_info.track_forward_angles)):
            angle = sensing_info.track_forward_angles[i]
            cur_dir = angle
            if cur_dir != 0:
                cur_dir /= abs(cur_dir)
            idx = 1
            
            angle_sum += angle
            if second:
                idx += 2
            if angle_sum < 20 and angle > -20 and direction[2] == 0:
                direction[0] += 1
            else:
                if angle * direction[idx] < 0 or direction[idx+1] >= 5 or direction[idx] > 100:
                    if second:
                        break
                    second = True
                if direction[1] == 0:
                    direction[1] = angle_sum
                    direction[1] += direction[0]
                    for a in range(0,i):
                        if sensing_info.track_forward_angles[i-a] * angle_sum > 0 and abs(sensing_info.track_forward_angles[i-a]) > 5:
                            direction[0] -= 1
                        else:
                            break
                direction[idx] += angle
                direction[idx+1] += 1
                
        return direction
    
    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #
        #self.is_debug = True
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

        # Moving straight forward

        steer_sum = 0
        dist_sum = 0
        
        #for i in range(0,20):
        #    steer_sum += (200-sensing_info.distance_to_way_points[i]) * sensing_info.track_forward_angles[i]
        #    dist_sum += 200-sensing_info.distance_to_way_points[i]

        #print(steer_sum/dist_sum)

        direction_info = self.merge_direction(sensing_info)
        #car_controls.steering = -sensing_info.moving_angle/90.0 + steer_avg + forward_five
        #print(car_controls.steering)
        car_controls.steering = 0
        #print(direction_info)
        car_controls.steering -= sensing_info.moving_angle/90
        
        curve_start = (sensing_info.speed-80) // 10

        if curve_start > 5:
            curve_start = 5
        if curve_start < 1:
            curve_start = 1
        
        #print(curve_start, direction_info)
        if direction_info[0] > curve_start:
            #print('hihi')
            #print(-sensing_info.to_middle -direction_info[1][0]*4)
            if direction_info[2] > 0:
                #avg_curve = abs((direction_info[1]+direction_info[3])/(90*(direction_info[2]+direction_info[4])))
                avg_curve = abs(direction_info[1]/(60*direction_info[2]))
                
                if avg_curve > 4:
                    avg_curve = 4
                #modifier = 1+avg_curve
                modifier = 4+avg_curve
                if direction_info[1] < 0:
                    modifier *= -1
                car_controls.steering += math.atan((-sensing_info.to_middle-modifier)/((direction_info[0])*10))*180/math.pi/90
            else:
                car_controls.steering += math.atan(-sensing_info.to_middle/(direction_info[0]*10))*180/math.pi/90
        #elif direction_info[0] > curve_start-2:
        #    car_controls.steering += math.atan(-sensing_info.to_middle/(direction_info[0]*10))*180/math.pi/150
        else:
            #print(direction_info)
            next_forward = sum(sensing_info.track_forward_angles[:direction_info[0]+1])
            car_controls.steering += (direction_info[1]/90 + next_forward/90)/(direction_info[2]+direction_info[0]+1)
            #car_controls.steering += (direction_info[1]/90)/(direction_info[2])
            #car_controls.steering += next_four/90
            #car_controls.steering += direction_info[1]/(90*direction_info[2])
            if direction_info[4] > 0:
                avg_curve = abs(direction_info[3]/(60*direction_info[4]))
                
                if avg_curve > 4:
                    avg_curve = 4
                #modifier = 1+avg_curve
                modifier = 4+avg_curve
                if direction_info[3] < 0:
                    modifier *= -1
                #print(math.atan((-sensing_info.to_middle/2 - modifier)/(direction_info[2]*10))*180/math.pi/90)
                car_controls.steering += math.atan((-sensing_info.to_middle - modifier)/(direction_info[2]*10))*180/math.pi/90
            else:
                car_controls.steering += math.atan(-sensing_info.to_middle/(direction_info[2]*10))*180/math.pi/90

        #next_angle = sensing_info.moving_angle+car_controls.steering*90
        #next_to_middle = math.sin(next_angle*math.pi/180)*sensing_info.speed/10
        #print(next_to_middle)
        #car_controls.steering += math.atan(-next_to_middle/100)*180/math.pi/90

        car_controls.throttle = 1
        car_controls.brake = 0
        if abs(car_controls.steering)*sensing_info.speed > 60:
            car_controls.throttle = 0
            car_controls.brake = abs(car_controls.steering)/1.5
            
        #print('steer', car_controls.steering, 'break', car_controls.brake)
            
        if self.is_debug:
            print("[MyCar] steering:{}, throttle:{}, brake:{}"\
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
