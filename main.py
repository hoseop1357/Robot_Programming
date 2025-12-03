#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# ==============================================================================
# 1. 설정 및 초기화
# ==============================================================================
ev3 = EV3Brick()

# 모터 설정
grap_motor = Motor(Port.C)
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)

# 센서 설정
left_sensor = ColorSensor(Port.S1)
right_sensor = ColorSensor(Port.S4)
object_detector = ColorSensor(Port.S2)
ultra_sensor = UltrasonicSensor(Port.S3)

# 로봇 제어 상수
WHEEL_DIAMETER = 56
AXLE_TRACK = 114
DRIVE_SPEED = 150
TURN_SPEED = 100

# 라인 트레이싱 설정
BLACK_THRESHOLD = 25    
kp = 0.8

robot = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)
robot.settings(straight_speed=DRIVE_SPEED, turn_rate=TURN_SPEED)

# ==============================================================================
# 2. 로봇 상태 및 지도 관리
# ==============================================================================
DIR_N, DIR_E, DIR_S, DIR_W = 1, 2, 3, 4

NODES = {
    1: (0, 0), 2: (1, 0), 3: (2, 0),
    5: (0, 1), 6: (1, 1), 7: (2, 1),
    9: (0, 2), 10: (1, 2), 11: (2, 2)
}

current_x = 0
current_y = 0
current_dir = DIR_E

# ==============================================================================
# 3. 유틸리티 함수
# ==============================================================================

def get_color():
    """RGB 값을 읽어 빨강/파랑 판별"""
    rgb = object_detector.rgb()
    r, g, b = rgb
    print("RGB Detect: R:{} G:{} B:{}".format(r,g,b))
    
    if r > 10 and r > b + 5:
        return Color.RED
    elif b > 15 and b > r + 5:
        return Color.BLUE
    return None

def grab_object():
    print("Action: Grab")
    grap_motor.run_time(500, 1500)
    wait(500)

def release_object():
    print("Action: Release")
    grap_motor.run_time(-500, 1500)
    wait(500)

def is_intersection():
    """양쪽 센서가 모두 검은색이면 교차로(라인)"""
    left_val = left_sensor.reflection()
    right_val = right_sensor.reflection()
    if left_val < BLACK_THRESHOLD and right_val < BLACK_THRESHOLD:
        return True
    return False

# ==============================================================================
# 4. 주행 함수
# ==============================================================================

def follow_line_until_event(check_obstacle=True):
    """
    라인을 따라가다가 교차로를 만나면 멈추고 'INTERSECTION' 반환
    """
    while True:
        # 1. 교차로 감지
        if is_intersection():
            robot.stop()
            robot.straight(45) # 교차로 중앙 정렬
            return "INTERSECTION"

        # 2. 물체 감지 (탐색 모드일 때만)
        if check_obstacle:
            dist = ultra_sensor.distance()
            if dist < 80:
                robot.stop()
                return "OBJECT"
            
        # 3. 라인 트레이싱
        l_val = left_sensor.reflection()
        r_val = right_sensor.reflection()
        
        error = l_val - r_val
        turn_rate = error * kp
        robot.drive(DRIVE_SPEED, turn_rate)
        wait(10)

def turn_absolute(target_dir):
    global current_dir
    diff = target_dir - current_dir
    
    if diff == 3: diff = -1
    elif diff == -3: diff = 1
    elif diff in [2, -2]: diff = 2
    
    angle = 0
    if diff == 1: angle = 90
    elif diff == -1: angle = -90
    elif diff == 2: angle = 180
    
    if angle != 0:
        robot.turn(angle)
    
    current_dir = target_dir
    wait(200)

def move_n_grid(n, check_obstacle=True):
    """
    n칸(n개의 교차로)만큼 이동.
    """
    for _ in range(n):
        status = follow_line_until_event(check_obstacle)
        if status == "OBJECT":
            return "OBJECT"
    return "ARRIVED"

def move_manhattan(target_node_id, check_obstacle=True):
    """
    현재 좌표에서 목표 노드 좌표로 이동 (1번 노드 복귀용)
    """
    global current_x, current_y
    target_x, target_y = NODES[target_node_id]
    
    # X축 이동
    dx = target_x - current_x
    if dx != 0:
        wanted_dir = DIR_E if dx > 0 else DIR_W
        turn_absolute(wanted_dir)
        status = move_n_grid(abs(dx), check_obstacle)
        if status == "OBJECT": return "OBJECT"
        current_x += dx

    # Y축 이동
    dy = target_y - current_y
    if dy != 0:
        wanted_dir = DIR_N if dy > 0 else DIR_S
        turn_absolute(wanted_dir)
        status = move_n_grid(abs(dy), check_obstacle)
        if status == "OBJECT": return "OBJECT"
        current_y += dy
        
    return "ARRIVED"

# ==============================================================================
# 5. [수정됨] 하드코딩 배달 로직 (1번 노드에서 시작)
# ==============================================================================

def deliver_hardcoded(color_type):
    """
    1번 노드에 도착한 상태에서 실행됩니다.
    무조건 정해진 순서(하드코딩)대로 움직여서 배달합니다.
    """
    print("Status: Hardcoded Delivery Start -> " + color_type)
    
    # --------------------------------------------------------------------------
    # 단계 1: 1번 노드 -> 복도 진입 (공통 동작)
    # --------------------------------------------------------------------------
    # 1. 서쪽(Start/Zone 방향)을 바라봅니다.
    turn_absolute(DIR_W)
    
    # 2. 한 칸 전진하여 그리드를 빠져나갑니다. (Start영역 옆 도착)
    print("Step 1: Exit Grid (Move 1 block West)")
    move_n_grid(1, check_obstacle=False)
    
    # --------------------------------------------------------------------------
    # 단계 2: 남쪽으로 회전 (공통 동작)
    # --------------------------------------------------------------------------
    print("Step 2: Face South")
    turn_absolute(DIR_S) 
    
    # --------------------------------------------------------------------------
    # 단계 3: 색상별 층수 이동 (분기)
    # --------------------------------------------------------------------------
    if color_type == 'Red':
        print("Step 3 (Red): Go 1 block South")
        # 빨강은 Start 바로 아래 -> 1칸 이동
        move_n_grid(1, check_obstacle=False)
        
    elif color_type == 'Blue':
        print("Step 3 (Blue): Go 2 blocks South")
        # 파랑은 빨강보다 더 아래 -> 2칸 이동
        move_n_grid(2, check_obstacle=False)
        
    # --------------------------------------------------------------------------
    # 단계 4: 서쪽 회전 후 방 안으로 진입
    # --------------------------------------------------------------------------
    print("Step 4: Turn West and Enter Zone")
    turn_absolute(DIR_W) # 방 쪽을 바라봄
    
    # [중요] 방 안으로 들어갈 때는 'move_n_grid' 대신 'robot.straight'를 추천합니다.
    # 방 안에는 멈출 수 있는 교차로(검은선)가 없을 수도 있기 때문입니다.
    # 요청하신 "한 Grid 직진"을 거리(20cm 정도)로 환산하여 진입합니다.
    robot.straight(200) 
    
    # --------------------------------------------------------------------------
    # 단계 5: 내려놓고 복귀
    # --------------------------------------------------------------------------
    print("Step 5: Release and Retreat")
    release_object()     
    robot.straight(-200)  # 들어온 만큼 뒤로 나옴
    
    # 다음 동작을 위해 다시 동쪽(복도) 방향을 바라봄 (선택사항)
    turn_absolute(DIR_E)

# ==============================================================================
# 6. 메인 실행
# ==============================================================================

def main():
    global current_x, current_y, current_dir
    ev3.speaker.beep()
    print("System Started")
    
    # 시작점 -> 1번 노드 진입
    follow_line_until_event(check_obstacle=True)
    follow_line_until_event(check_obstacle=True)
    
    current_x, current_y = 0, 0
    current_dir = DIR_E
    
    search_list = [1, 2, 3, 5, 6, 7, 9, 10, 11]
    found_item = None
    
    # ------------------------------------------------------------------
    # 1. 탐색 및 집기
    # ------------------------------------------------------------------
    for target in search_list:
        print("Target Node: {}".format(target))
        status = move_manhattan(target, check_obstacle=True)
        
        if status == "OBJECT":
            print("Status: Object Detected")
            wait(500)
            robot.straight(70) # 접근
            grab_object()      # 집기
            wait(500)
            
            detected_color = get_color()
            if detected_color == Color.RED:
                found_item = 'Red'
                print("Color Detected: RED")
            elif detected_color == Color.BLUE:
                found_item = 'Blue'
                print("Color Detected: BLUE")
            else:
                found_item = 'Unknown'
                print("Color Detected: Unknown")
            
            break 
            
    # ------------------------------------------------------------------
    # 2. 1번 노드 복귀 -> 하드코딩 배달
    # ------------------------------------------------------------------
    if found_item:
        print("Returning to Hub (Node 1)...")
        
        # [핵심] 어디에 있든 일단 1번 노드(0,0)로 돌아옵니다.
        move_manhattan(1, check_obstacle=False)
        
        # 1번 노드 도착 완료. 이제 하드코딩된 경로로 이동합니다.
        if found_item == 'Red':
            deliver_hardcoded('Red')
        elif found_item == 'Blue':
            deliver_hardcoded('Blue')
        else:
            print("Status: Unknown color, releasing object.")
            release_object() 
            
    else:
        print("Result: Not Found")
        
    ev3.speaker.beep()
    print("System Finished")

if __name__ == '__main__':
    main()