import time
import picamera
import numpy as np
import cv2
import RPi.GPIO as GPIO

button_pin = 27 #GPIO 27
led_red_pin = 22 #GPIO 22
led_green_pin = 23 #GPIO 23

GPIO.setmode(GPIO.BCM)

GPIO.setup(button_pin, GPIO.IN)
GPIO.setup(led_red_pin, GPIO.OUT)
GPIO.setup(led_green_pin, GPIO.OUT)

GPIO.output(led_red_pin, True)  # 초기에는 빨간색 LED 켜기
GPIO.output(led_green_pin, False)  # 초기에는 초록색 LED 켜기

buttonInputPrev = False # 버튼눌림확인
motion_detection_enabled = False # 동작감지활성화여부

# 카메라 설정
camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 24
raw_capture = np.empty((480, 640, 3), dtype=np.uint8)

# 움직임 감지에 필요한 변수 초기화
last_frame = None
motion_detected = False

# 움직임 감지 함수
def detect_motion(frame):
    global last_frame, motion_detected

    # 흑백 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)

    # 초기 프레임이 없는 경우 현재 프레임을 저장하고 종료
    if last_frame is None:
        last_frame = gray
        return

    # 이전 프레임과 현재 프레임의 차이 계산
    frame_delta = cv2.absdiff(last_frame, gray)
    thresh = cv2.threshold(frame_delta, 30, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)

    # 이진 이미지에서 윤곽선 찾기
    contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 움직임이 있는 경우
    if len(contours) > 0:
        motion_detected = True
        print("동작이 감지되었습니다")
        GPIO.output(led_green_pin, True)
        GPIO.output(led_red_pin, False)
    # 움직임이 없는경우
    else:
        motion_detected = False
        GPIO.output(led_green_pin, False)
        GPIO.output(led_red_pin, True)

    # 현재 프레임을 이전 프레임으로 업데이트
    last_frame = gray

try:
    while True:
        buttonInput = GPIO.input(button_pin)

        if buttonInput and not buttonInputPrev:
            motion_detection_enabled = not motion_detection_enabled  # 동작 감지 기능 활성화/비활성화 전환
            if motion_detection_enabled:
                print("[동작감지 활성화]")
            else:
                print("[동작감지 비활성화]")

        buttonInputPrev = buttonInput

        if motion_detection_enabled:
            # 움직임 감지 기능 활성화
            camera.capture(raw_capture, format="bgr")
            frame = raw_capture
            detect_motion(frame)
            raw_capture = np.empty((480, 640, 3), dtype=np.uint8)

        time.sleep(0.1)

except KeyboardInterrupt:
    # 프로그램 종료 시 카메라 스트리밍 정지
    camera.stop_preview()

GPIO.cleanup()
