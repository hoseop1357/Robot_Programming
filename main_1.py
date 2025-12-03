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

# 센서 기준값
INTERSECTION_LIMIT = 20 

# 튜닝 파라미터
SPEED = 150        
KP = 0.5          
OBJECT_DETECT_DIST = 80   
GRAB_APPROACH_DIST = 70   
RECOVERY_DIST = -30
PUSH_DIST = 100       

# 상태 변수
current_x, current_y = 0, 0 
current_dir = E
OBSTACLE_NODES = set()

# --- [기본 함수] ---

def dual_sensor_line_follow():
    left_val = left_sensor.reflection()
    right_val = right_sensor.reflection()
    
    if left_val < 25 and right_val < 25:
        robot.drive(SPEED, 0)
        return

    error = left_val - right_val
    turn_rate = KP * error
    robot.drive(SPEED, turn_rate)

def n_move(n):
    """n칸(점 n개) 이동 함수"""
    robot.drive(SPEED, 0)
    wait(300) 
    
    for i in range(n):
        while True:
            # 물체 감지
            if ultra_sensor.distance() < OBJECT_DETECT_DIST:
                robot.stop()
                return "OBJECT"
            
            # 교차로 감지
            l_ref = left_sensor.reflection()
            r_ref = right_sensor.reflection()
            if l_ref < INTERSECTION_LIMIT and r_ref < INTERSECTION_LIMIT:
                break 
            
            dual_sensor_line_follow()
            wait(5)
            
        robot.drive(SPEED, 0)
        wait(200)
        
    robot.stop()
    robot.straight(-10) 
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
        wait(300) 
        while left_sensor.reflection() > INTERSECTION_LIMIT:
            robot.drive(0, turn_speed)
            wait(10)
        robot.stop()
        
    elif diff == 3: 
        robot.drive(0, -turn_speed)
        wait(300)
        while right_sensor.reflection() > INTERSECTION_LIMIT:
            robot.drive(0, -turn_speed)
            wait(10)
        robot.stop()
        
    elif diff == 2: 
        robot.turn(180)
        
    current_dir = target_dir

def move_between_nodes(target_dir):
    """노드 간 이동 (2칸 이동)"""
    global current_x, current_y, current_dir
    
    turn_to(target_dir)
    res = n_move(2)
    
    if res == "OBJECT":
        return "OBJECT"

    # 정상 도착 시 좌표 업데이트
    if current_dir == N: current_y -= 2
    elif current_dir == E: current_x += 2
    elif current_dir == S: current_y += 2
    elif current_dir == W: current_x -= 2
    
    print("[POS] Arrived at ({}, {})".format(current_x, current_y))
    return "ARRIVED"

def get_color():
    rgb = object_detector.rgb()
    r, g, b = rgb
    if r > 10 and r > b + 8: return Color.RED
    elif b > 15 and b > r + 10: return Color.BLUE
    return None

# --- [로직] BFS 경로 탐색 ---

def bfs_path_find(start_node, target_func):
    """BFS 최단 경로 탐색"""
    queue = deque([(start_node, [])])
    visited = set([start_node])
    
    while queue:
        (cx, cy), path = queue.popleft()
        
        if target_func(cx, cy):
            return path
            
        # 4방향 탐색
        neighbors = [(0,-2,N), (2,0,E), (0,2,S), (-2,0,W)]
        for dx, dy, d in neighbors:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                # [맵 벽 체크] (3,2)와 (3,4)는 막혀있음
                is_wall = False
                if (cx == 2 and nx == 4) and (cy == 2 or cy == 4): is_wall = True
                if (cx == 4 and nx == 2) and (cy == 2 or cy == 4): is_wall = True
                
                if not is_wall and (nx, ny) not in visited and (nx, ny) not in OBSTACLE_NODES:
                    visited.add((nx, ny))
                    queue.append(((nx, ny), path + [d]))
    return None

def bfs_search_target(visited_global):
    """탐색 모드: (2,0) 또는 (4,0) 이상인 영역 중 미방문 노드 찾기"""
    def target(x, y):
        # x>=4 (탐색 격자) 이거나 x==2 (복도) 중 방문 안 한 곳
        return x >= 2 and (x, y) not in visited_global and (x, y) not in OBSTACLE_NODES
    return bfs_path_find((current_x, current_y), target)

# --- [로직] 하드코딩 배달 ---

def deliver(color):
    """복귀 및 배달 함수 (정렬 기능 추가)"""
    global current_x, current_y, current_dir
    
    print("[RETURN] Returning to Split Point (2,0)...")
    
    # 1. [현장 수습] 뒤로 물러나기
    robot.straight(RECOVERY_DIST) 
    
    # 2. [180도 회전]
    robot.turn(180)
    
    # ==========================================================
    # ★★★ [추가됨] 라인 정렬 (Alignment) 코드 ★★★
    # 180도 회전 후 로봇이 비스듬하면 똑바로 맞춰줍니다.
    # 원리: 왼쪽/오른쪽 센서 값이 비슷해질 때까지 제자리 회전
    print("[ALIGN] Aligning to line...")
    align_timer = StopWatch()
    
    while align_timer.time() < 1500: # 최대 1.5초 동안만 정렬 시도 (무한루프 방지)
        l_val = left_sensor.reflection()
        r_val = right_sensor.reflection()
        error = l_val - r_val
        
        # 오차가 작으면(5 이내) 정렬 완료로 판단하고 정지
        if abs(error) < 5:
            robot.stop()
            break
            
        # 오차만큼 제자리 회전 (속도 0, 회전만)
        # KP값을 작게(1.0) 주어 부드럽게 정렬
        robot.drive(0, error * 1.0)
        wait(10)
    robot.stop()
    # ==========================================================

    # 방향 변수 업데이트
    if current_dir == N: current_dir = S
    elif current_dir == S: current_dir = N
    elif current_dir == E: current_dir = W
    elif current_dir == W: current_dir = E
    print("[POS] 180 Turn Complete. Facing: {}".format(DIR_NAME[current_dir]))
    
    # 3. [BFS 복귀] 분기점(2,0)으로 이동
    path_to_split = bfs_path_find((current_x, current_y), lambda x, y: x==2 and y==0)
    
    if path_to_split is None:
        ev3.speaker.say("No Path")
        return

    for d in path_to_split:
        res = move_between_nodes(d)
        if res == "OBJECT":
            ev3.speaker.say("Error")
            break
            
    print("[POS] Arrived at Split Point (2,0)")
    ev3.speaker.beep()
    
    # 4. [배달]
    turn_to(S)
    robot.stop(); wait(500)
    
    if color == Color.RED:
        print("[DELIVER] RED -> (0,2)")
        n_move(2)
        turn_to(W); release()
        robot.straight(PUSH_DIST); robot.straight(-PUSH_DIST)
        turn_to(N); n_move(2)
        
    elif color == Color.BLUE:
        print("[DELIVER] BLUE -> (0,4)")
        n_move(4)
        turn_to(W); release()
        robot.straight(PUSH_DIST); robot.straight(-PUSH_DIST)
        turn_to(N); n_move(4)
        
    else: 
        print("[DELIVER] Unknown -> RED")
        n_move(2)
        turn_to(W); release()
        robot.straight(PUSH_DIST); robot.straight(-PUSH_DIST)
        turn_to(N); n_move(2)

    # 5. [상태 재설정]
    current_x, current_y = 2, 0
    turn_to(E)
    print("[STATUS] Ready at (2,0)")

# --- [메인] ---
release()
def main():
    global current_x, current_y, current_dir
    visited = set()
    ev3.speaker.say("Start")
    
    # 1. 초기 진입: Start(0,0) -> 분기점(2,0)
    print("[INIT] Moving to Split Point (2,0)")
    n_move(2) # 점 2개 이동
    current_x, current_y = 2, 0
    visited.add((2, 0))
    print("[POS] Initialized at (2,0)")
    
    while True:
        # 2. 탐색 경로 계산 (BFS)
        path = bfs_search_target(visited)
        
        if not path:
            ev3.speaker.say("End")
            print("[FINISH] Exploration Complete")
            break
            
        object_found = False
        
        # 3. 경로 이동
        for d in path:
            res = move_between_nodes(d)
            
            if res == "OBJECT":
                object_found = True
                
                # 장애물 위치 추정 (현재 위치에서 가려던 방향 2칸 앞)
                ox, oy = current_x, current_y
                if d == N: oy -= 2
                elif d == E: ox += 2
                elif d == S: oy += 2
                elif d == W: ox -= 2
                
                print("[OBS] Detected at ({}, {})".format(ox, oy))
                break
            
            visited.add((current_x, current_y))
            
        # 4. 물체 발견 및 배달
        if object_found:
            # 잡기
            robot.straight(GRAB_APPROACH_DIST)
            grab()
            c = get_color()
            
            # 잡았으니 장애물이 아님 -> 탐색 대상에서 제외하지 않고 방문 처리 해야 함?
            # 일단 장애물 리스트에는 넣지 않거나 넣었다 뺌 (여기선 안 넣음)
            
            # [복귀] BFS로 분기점(2,0)까지 이동
            print("[RETURN] Returning to Split Point (2,0)...")
            
            # 현장 수습: 뒤로 물러나 180도 회전
            robot.straight(RECOVERY_DIST)
            turn_to((current_dir + 2 - 1) % 4 + 1)
            
            # (2,0)으로 가는 경로 계산
            path_to_split = bfs_path_find((current_x, current_y), lambda x, y: x==2 and y==0)
            
            if path_to_split:
                for d in path_to_split:
                    move_between_nodes(d)
            
            print("[POS] Back at Split Point (2,0)")
            ev3.speaker.beep()
            
            # [배달] 하드코딩 실행
            deliver(c)
            
            # 배달 후 (2,0)은 방문 완료
            visited.add((2, 0))

if __name__ == "__main__":
    main()