#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, UltrasonicSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase

# [설정 1] 객체 및 포트 초기화
ev3 = EV3Brick()

# Sensors
left_sensor = ColorSensor(Port.S1)      # 라인트레이싱 (왼쪽)
right_sensor = ColorSensor(Port.S4)     # 라인트레이싱 (오른쪽)
object_detector = ColorSensor(Port.S2)  # 물체 색상 감지 (RGB)
ultra_sensor = UltrasonicSensor(Port.S3)# 장애물(물체) 거리 감지

# Motors
grab_motor = Motor(Port.C)              # 집게 모터
left_motor = Motor(Port.A)              # 주행 모터 (왼쪽)
right_motor = Motor(Port.D)             # 주행 모터 (오른쪽)

# DriveBase
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=114)

# [설정 2] 방향 상수 정의
N, E, S, W = 1, 2, 3, 4

# [설정 3] 맵 및 로봇 상태 전역 변수
GRID_W, GRID_H = 4, 3

# ★★★ [수정됨] 시작 좌표를 (0, 2)로 설정 (왼쪽 상단 1번 노드) ★★★
current_x, current_y = 0, 2 
current_dir = E 

# [설정 4] 사용자 측정 데이터 기반 상수
LINE_THRESHOLD = 50       
INTERSECTION_LIMIT = 15   

# ====================================================
# [설정 5] ★★★ 튜닝 파라미터 ★★★
# ====================================================
BASE_SPEED = 80           # 기본 주행 속도
LINE_KP = 0.6             # 핸들 민감도

# ★★★ [수정됨] 감지 거리가 너무 짧으면 충돌하므로 50~80mm 추천 ★★★
OBJECT_DETECT_DIST = 80   
GRAB_APPROACH_DIST = 30   # 잡기 전 접근 거리 (mm)

# [신규] 물체 잡은 후 동작 설정
POST_GRAB_MOVE_DIST = 0   
# ====================================================

# --- [기능 1] 기본 이동 및 회전 함수 ---

def turn_min(now_dir, target_dir):
    """최소 회전 각도 계산 및 수행"""
    if now_dir == target_dir:
        return target_dir
    diff = (target_dir - now_dir) % 4
    angle = [0, 90, 180, -90][diff]
    robot.turn(angle)
    return target_dir

def line_follow_step():
    """부드러운 라인 트레이싱"""
    error = right_sensor.reflection() - LINE_THRESHOLD
    turn_rate = -LINE_KP * error
    robot.drive(BASE_SPEED, turn_rate)

def move_one_block(target_dir):
    """한 칸 이동 및 교차로/물체 감지"""
    global current_x, current_y, current_dir
    
    # 1. 방향 회전
    current_dir = turn_min(current_dir, target_dir)
    
    # 2. 교차로 탈출
    robot.drive(BASE_SPEED, 0)
    wait(500) 
    
    found_object = False
    
    # 3. 주행 루프
    while True:
        # (1) 물체 감지
        if ultra_sensor.distance() < OBJECT_DETECT_DIST:
            ev3.speaker.beep()
            robot.stop()
            found_object = True
            break
            
        # (2) 교차로 감지
        left_val = left_sensor.reflection()
        right_val = right_sensor.reflection()
        
        if left_val < INTERSECTION_LIMIT and right_val < INTERSECTION_LIMIT:
            robot.stop()
            break
            
        line_follow_step()
        wait(10)
    
    if found_object:
        return "OBJECT_FOUND"

    # 4. 좌표 업데이트
    if current_dir == N: current_y += 1
    elif current_dir == E: current_x += 1
    elif current_dir == S: current_y -= 1
    elif current_dir == W: current_x -= 1
    
    # 교차로 중앙 정렬
    robot.straight(55) 
    return "ARRIVED"

def grab_object():
    ev3.speaker.beep()
    # 집게 조이기 (200)
    grab_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=50)

def release_object():
    # 집게 풀기 (-200)
    grab_motor.run_until_stalled(-200, then=Stop.COAST, duty_limit=50)

# --- [기능 2] RGB 색상 판별 함수 ---

def get_color_via_rgb():
    rgb_values = object_detector.rgb()
    r, g, b = rgb_values
    
    print("RGB:", r, g, b) 
    
    if r > 10 and r > b + 8: 
        return Color.RED
    elif b > 15 and b > r + 10:
        return Color.BLUE
    else:
        return None 

# --- [기능 3] 알고리즘 (BFS & Manhattan) ---

def bfs_search(visited):
    queue = [(current_x, current_y)]
    path_map = {(current_x, current_y): []}
    visited_bfs = set([(current_x, current_y)])
    
    while queue:
        cx, cy = queue.pop(0)
        curr_path = path_map[(cx, cy)]
        
        if (cx, cy) not in visited:
            return (cx, cy), curr_path
            
        neighbors = [
            (cx, cy+1, N), (cx+1, cy, E), (cx, cy-1, S), (cx-1, cy, W)
        ]
        
        for nx, ny, move_dir in neighbors:
            if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                if (nx, ny) not in visited_bfs:
                    visited_bfs.add((nx, ny))
                    queue.append((nx, ny))
                    path_map[(nx, ny)] = curr_path + [move_dir]
    return None, []

def return_manhattan(color):
    global current_dir
    
    # 인식된 색상이 없으면(None) 일단 Red Zone(임의)으로 보냅니다.
    if color == Color.RED:
        target_x, target_y = 0, 2 
    elif color == Color.BLUE:
        target_x, target_y = 0, 0 
    else:
        target_x, target_y = 0, 2 

    ev3.speaker.say("Return")

    # --- [복귀 로직] ---
    # 1. 잡은 후 전진 (또는 후진)
    robot.straight(POST_GRAB_MOVE_DIST)
    
    # 2. 180도 회전
    robot.turn(180)
    
    # 3. 로봇 방향 변수 반대로 업데이트
    if current_dir == N: current_dir = S
    elif current_dir == S: current_dir = N
    elif current_dir == E: current_dir = W
    elif current_dir == W: current_dir = E
    
    # 4. 원래 교차로까지 되돌아감 (라인 트레이싱)
    while True:
        if left_sensor.reflection() < INTERSECTION_LIMIT and right_sensor.reflection() < INTERSECTION_LIMIT:
            robot.stop()
            break
        line_follow_step()
        wait(10)
        
    robot.straight(55) # 교차로 정렬

    # 5. 맨해튼 거리 이동
    dx = target_x - current_x
    if dx != 0:
        move_dir = E if dx > 0 else W
        while current_x != target_x:
            move_one_block(move_dir)
            
    dy = target_y - current_y
    if dy != 0:
        move_dir = N if dy > 0 else S
        while current_y != target_y:
            move_one_block(move_dir)
            
    turn_min(current_dir, W) 
    release_object()
    
    robot.straight(-100)
    turn_min(W, E) 

# --- [메인] 실행 로직 ---

def main():
    global current_x, current_y
    visited_nodes = set()
    
    ev3.speaker.say("Start")
    
    # 1. 초기 진입 (Start -> 1번 노드)
    while True:
        if left_sensor.reflection() < INTERSECTION_LIMIT and right_sensor.reflection() < INTERSECTION_LIMIT:
            robot.stop()
            break
        line_follow_step()
        wait(10)
    
    robot.straight(55) 
    visited_nodes.add((current_x, current_y))

    while True:
        # 2. BFS 탐색
        target_node, paths = bfs_search(visited_nodes)
        
        if target_node is None:
            ev3.speaker.say("End")
            break
            
        # 3. 이동
        object_detected = False
        for move_dir in paths:
            result = move_one_block(move_dir)
            if result == "OBJECT_FOUND":
                object_detected = True
                break
        
        # 4. 물체 처리
        if not object_detected:
            visited_nodes.add((current_x, current_y))
        else:
            # 물체 접근
            robot.straight(GRAB_APPROACH_DIST)
            
            grab_object()
            wait(500)
            
            detected_color = get_color_via_rgb()
            
            if detected_color == Color.RED:
                ev3.speaker.say("Red")
            elif detected_color == Color.BLUE:
                ev3.speaker.say("Blue")
            else:
                ev3.speaker.say("Unknown")
            
            # 여기서 수정된 복귀 함수 호출 (전진 -> 180도 회전 -> 이동)
            return_manhattan(detected_color)
            
            visited_nodes.add((current_x, current_y))

if __name__ == "__main__":
    main()