#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from collections import deque

# [설정] 객체 초기화
ev3 = EV3Brick()
left_sensor = ColorSensor(Port.S1)
right_sensor = ColorSensor(Port.S4)
object_detector = ColorSensor(Port.S2)
ultra_sensor = UltrasonicSensor(Port.S3)

grab_motor = Motor(Port.C)
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=114)

# [상수] 좌표계 (9x5 Grid)
GRID_W, GRID_H = 9, 5
N, E, S, W = 1, 2, 3, 4
DIR_NAME = {1: 'N', 2: 'E', 3: 'S', 4: 'W'}

# [설정 4] 센서 기준값
INTERSECTION_LIMIT = 50 

# 튜닝 파라미터
SPEED = 100      
KP = 1.2 
KD = 3.0             
OBJECT_DETECT_DIST = 80   
GRAB_APPROACH_DIST = 70   
RECOVERY_DIST = -20
PUSH_DIST = 100   
# [추가됨] 마지막 오차를 기억할 변수
last_error = 0    

# 상태 변수
current_x, current_y = 0, 0 
current_dir = E
OBSTACLE_NODES = set()

# --- [기본 함수] ---

def dual_sensor_line_follow():
    """
    [PD 제어 + 가변 속도 + 이탈 방지] 통합 주행 함수
    """
    global last_error # 전역 변수 사용 필수
    
    left_val = left_sensor.reflection()
    right_val = right_sensor.reflection()
    
    # 1. [교차로 처리] 양쪽 다 검정이면 직진 (기존 유지)
    if left_val < 25 and right_val < 25:
        robot.drive(SPEED, 0)
        return

    # 2. [이탈 방지] 양쪽 다 흰색(라인 놓침)인 경우
    # 방금 전 에러(last_error)가 양수였다면 오른쪽, 음수였다면 왼쪽으로 더 강하게 꺾음
    if left_val > 60 and right_val > 60:
        if last_error > 0: error = 60 # 오른쪽으로 더 꺾게 유도
        else: error = -60             # 왼쪽으로 더 꺾게 유도
    else:
        # 평소 주행 (왼쪽 - 오른쪽)
        error = left_val - right_val

    # 3. [PD 제어 계산]
    # P항: 현재 에러
    # D항: (현재 에러 - 이전 에러) -> 변화량(속도)을 제어
    derivative = error - last_error
    turn_rate = (KP * error) + (KD * derivative)
    
    # 4. [가변 속도]
    # 에러가 클수록(많이 꺾어야 할수록) 직진 속도를 줄임
    # 예: 에러가 50이면 -> 50 * 0.8 = 40만큼 감속
    correction_speed = SPEED - (abs(error) * 0.8)
    if correction_speed < 20: correction_speed = 20 # 최소 속도 보장
    
    # 5. 주행 및 상태 저장
    robot.drive(correction_speed, turn_rate)
    last_error = error

def n_move(n, detection_enabled=True):
    """
    [수정됨] detection_enabled가 False면 장애물 감지를 하지 않음
    """
    global current_x, current_y, current_dir
    
    robot.drive(SPEED, 0)
    wait(300) 
    
    for i in range(n):
        while True:
            # 1. 물체 감지 (옵션이 켜져 있을 때만 수행)
            if detection_enabled:
                if ultra_sensor.distance() < OBJECT_DETECT_DIST:
                    robot.stop()
                    return "OBJECT"
            
            # 2. 교차로 감지
            l_ref = left_sensor.reflection()
            r_ref = right_sensor.reflection()
            if l_ref < INTERSECTION_LIMIT or r_ref < INTERSECTION_LIMIT:
                wait(10)
                if left_sensor.reflection() < INTERSECTION_LIMIT + 5 or right_sensor.reflection() < INTERSECTION_LIMIT + 5:
                    break 
            
            # 3. 주행
            dual_sensor_line_follow()
            wait(5)
            
        # 좌표 업데이트
        if current_dir == N: current_y -= 1
        elif current_dir == E: current_x += 1
        elif current_dir == S: current_y += 1
        elif current_dir == W: current_x -= 1
        
        # 짝수 좌표(Node)일 때만 출력
        if current_x % 2 == 0 and current_y % 2 == 0:
            ev3.speaker.beep() 
            print("[POS] Arrived at Node ({}, {})".format(current_x, current_y))
        
        robot.drive(SPEED, 0)
        wait(250) 
        
    robot.stop() 
    return "ARRIVED"

def grab():
    ev3.speaker.beep()
    grab_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=50)

def release():
    grab_motor.run_until_stalled(-200, then=Stop.COAST, duty_limit=50)

def turn_to(target_dir):
    global current_dir
    if current_dir == target_dir: return
    
    diff = (target_dir - current_dir) % 4
    turn_speed = 100
    
    if diff == 1: 
        robot.drive(0, turn_speed)
        wait(500) # 회전 시간 조금 늘림 (라인 확실히 벗어나게)
        while left_sensor.reflection() > INTERSECTION_LIMIT:
            robot.drive(0, turn_speed)
            wait(10)
        robot.stop()
        
    elif diff == 3: 
        robot.drive(0, -turn_speed)
        wait(500)
        while right_sensor.reflection() > INTERSECTION_LIMIT:
            robot.drive(0, -turn_speed)
            wait(10)
        robot.stop()
        
    elif diff == 2: 
        robot.turn(180)
        
    current_dir = target_dir

def move_between_nodes(target_dir, detection_enabled=True):
    """[수정됨] 감지 옵션을 n_move로 전달"""
    turn_to(target_dir)
    return n_move(2, detection_enabled)

def get_color():
    rgb = object_detector.rgb()
    r, g, b = rgb
    if r > 10 and r > b + 8: return Color.RED
    elif b > 15 and b > r + 10: return Color.BLUE
    return None

# --- [로직] BFS & 배달 ---

def bfs_path_find(start_node, target_func):
    queue = deque([(start_node, [])])
    visited = set([start_node])
    
    while queue:
        (cx, cy), path = queue.popleft()
        if target_func(cx, cy):
            return path
        
        neighbors = [(0,-2,N), (2,0,E), (0,2,S), (-2,0,W)]
        for dx, dy, d in neighbors:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                is_wall = False
                if cx == 2 and nx == 4 and (cy == 2 or cy == 4): is_wall = True
                if cx == 4 and nx == 2 and (cy == 2 or cy == 4): is_wall = True
                
                if not is_wall and (nx, ny) not in visited and (nx, ny) not in OBSTACLE_NODES:
                    visited.add((nx, ny))
                    queue.append(((nx, ny), path + [d]))
    return None

def bfs_next_path(visited_global):
    def target(x, y):
        # x>=4 영역 탐색
        return x >= 4 and (x, y) not in visited_global and (x, y) not in OBSTACLE_NODES
    return bfs_path_find((current_x, current_y), target)

def deliver(color):
    """
    [수정됨] 정밀 복귀 로직 (라인 정렬 + 안전 복귀)
    """
    global current_x, current_y, current_dir
    
    print("[RETURN] Start Returning Procedure...")
    
    # 1. [현장 수습] 뒤로 물러나기
    robot.straight(RECOVERY_DIST) 
    
    # 2. [180도 회전]
    # 현재 방향의 반대 방향 계산
    target_dir = (current_dir + 2 - 1) % 4 + 1
    
    # 먼저 대략적으로 180도 회전
    robot.turn(180)
    
    # # ★★★ [추가] 라인 정렬 (Alignment) ★★★
    # # 회전 후 비뚤어진 자세를 센서로 바로잡음
    # # 왼쪽/오른쪽 센서 차이가 0이 될 때까지 제자리 회전
    # # (단, 시간 제한을 두어 무한루프 방지)
    # timer = StopWatch()
    # while timer.time() < 1500: # 최대 1.5초
    #     l = left_sensor.reflection()
    #     r = right_sensor.reflection()
    #     error = l - r
        
    #     # 오차가 작으면 정렬 완료
    #     if abs(error) < 5: break
        
    #     # 오차만큼 미세 회전 (P제어)
    #     robot.drive(0, error * 1.5)
    #     wait(10)
    # robot.stop()
    
    # 방향 변수 업데이트
    current_dir = target_dir
    print("[POS] 180 Turn & Align Complete. Facing: {}".format(DIR_NAME[current_dir]))
    
    # 3. [초기 위치 복귀] 
    # 물체 잡은 곳에서 가장 가까운 교차로(방금 지나온 노드)까지 이동
    # n_move를 쓰지 않고, 교차로 감지될 때까지 직진 트레이싱만 수행
    # (이미 노드 위에 있을 수도 있으므로 짧게 체크)
    while True:
        l = left_sensor.reflection()
        r = right_sensor.reflection()
        if l < INTERSECTION_LIMIT and r < INTERSECTION_LIMIT:
            break
        dual_sensor_line_follow()
        wait(5)
    robot.stop()
    
    # 현재 위치(노드)에서 분기점(2,0)까지 가는 경로 계산
    path_to_split = bfs_path_find((current_x, current_y), lambda x, y: x==2 and y==0)
    
    if path_to_split is None:
        ev3.speaker.say("No Path")
        return

    # 4. [BFS 복귀 주행]
    for d in path_to_split:
        # 물체 들고 있으므로 감지 끄고 이동
        res = move_between_nodes(d, detection_enabled=False)
        if res == "OBJECT":
            ev3.speaker.say("Error")
            break
            
    print("[POS] Arrived at Split Point (2,0)")
    ev3.speaker.beep()
    
    # 3. 배달 (하드코딩)
    robot.straight(-10)
    robot.turn(-90)
    current_dir = S
    ev3.speaker.say("Box Box")
    robot.stop(); wait(100)
    
    if color == Color.RED:
        print("[DELIVER] RED -> (0,2)")
        n_move(2)
        robot.turn(90); current_dir = W
        n_move(1)
        release()
        robot.straight(PUSH_DIST); robot.straight(-PUSH_DIST)
        robot.turn(180); current_dir = E
        ev3.speaker("Simply Lovely")
        n_move(1)
        robot.turn(-90); current_dir = N 
        n_move(2)
        
    elif color == Color.BLUE:
        print("[DELIVER] BLUE -> (0,4)")
        n_move(4)
        robot.turn(90)
        n_move(1); release()
        robot.straight(PUSH_DIST); robot.straight(-PUSH_DIST)
        robot.turn(180)
        n_move(1)
        turn_to(N); n_move(4)
        
    else: 
        print("[DELIVER] Unknown -> RED")
        n_move(2)
        turn_to(W); release()
        robot.straight(PUSH_DIST); robot.straight(-PUSH_DIST)
        turn_to(N); n_move(2)

    # 4. 상태 재설정
    current_x, current_y = 2, 0
    turn_to(E)
    print("[STATUS] Ready at (2,0)")

release()

def main():
    global current_x, current_y, current_dir
    visited = set()
    ev3.speaker.say("GoGoGo")
    
    # 1. 진입: Start(0,0) -> Node 1(4,0) (점 4개)
    # n_move(4)가 (1,0)->(2,0)[beep]->(3,0)->(4,0)[beep] 순으로 처리함
    n_move(4) 
    current_x, current_y = 4, 0
    visited.add((4, 0))
    print("[POS] Init at (4,0)")
    
    while True:
        # 2. 탐색
        path = bfs_next_path(visited)
        if not path:
            ev3.speaker.say("End")
            print("[FINISH] Exploration Complete")
            break
            
        object_found = False
        
        for d in path:
            res = move_between_nodes(d)
            if res == "OBJECT":
                object_found = True
                
                # 장애물 위치 (2칸 앞)
                ox, oy = current_x, current_y
                if d == N: oy -= 2
                elif d == E: ox += 2
                elif d == S: oy += 2
                elif d == W: ox -= 2
                
                OBSTACLE_NODES.add((ox, oy))
                print("[OBS] Detected at ({}, {})".format(ox, oy))
                
                # [중요] 물체 발견 시 좌표 강제 동기화 (거기 도착한 셈)
                current_x, current_y = ox, oy
                break
            
            visited.add((current_x, current_y))
            
        if object_found:
            robot.straight(GRAB_APPROACH_DIST)
            grab()
            c = get_color()
            
            # 잡은 위치(현재 위치)에서 장애물 해제
            if (current_x, current_y) in OBSTACLE_NODES: 
                OBSTACLE_NODES.remove((current_x, current_y))
            
            deliver(c)
            visited.add((2, 0)) # 분기점 방문

if __name__ == "__main__":
    main()