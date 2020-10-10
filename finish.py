import cv2
import numpy as np
import sys
import math
import timeit
import serial

serial_use = 1

serial_port = None
Read_RX = 0
receiving_exit = 1
threading_Time = 0.01

lower_red = np.array([150, 60, 60])
upper_red = np.array([180, 255, 255])
lower_blue = np.array([90, 70, 40])
upper_blue = np.array([120, 255, 255])
lower_green = np.array([50, 60, 80])
upper_green = np.array([80, 255, 255])
lower_yellow = np.array([0, 80, 110])
upper_yellow = np.array([30, 255, 255])


# -----------------------------------------------


# -----------------------------------------------
def TX_data_py2(ser, one_byte):  # one_byte= 0~255

    # ser.write(chr(int(one_byte)))          #python2.7
    ser.write(serial.to_bytes([one_byte]))  # python3


# -----------------------------------------------
def RX_data(ser):
    if ser.inWaiting() > 0:
        result = ser.read(1)
        RX = ord(result)
        return RX
    else:
        return 0


# -----------------------------------------------

def linetracer(res_yellow):
    cv2.imshow('line', res_yellow)

    line_number = 129

    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    dst = cv2.dilate(res_yellow, k)

    # imgray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)

    # cam = cv2.GaussianBlur(imgray, (3, 3), 0)  # 가우시안 블러

    ret, cam_binary = cv2.threshold(dst, 127, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)  # 이진화
    contours, hierarchy = cv2.findContours(cam_binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)  # 컨투어

    if len(contours) == 0:
        print("컨투어 0")

    else:
        contr = contours[0]
        x, y, w, h = cv2.boundingRect(contr)  # 최소한의 사각형 그려주는 함수
        cv2.rectangle(res_yellow, (x, y), (x + w, y + h), 255, 3)

        print(" x = %d, w = %d" % (x, w))
        print(' y = %d, h = %d' % (y, h))

        if (w >= 75 and w <= 120):
            print('직진')
            line_number = 111

        elif w >= 300:
            print('코너감지')
            line_number = 126

        elif x < 320 and w > 180 and w < 300:
            print('left 정렬')
            line_number = 114
        elif x > 320 and w > 180 and w < 300:
            print('right 정렬')
            line_number = 113

        else:
            print('라인이탈 -> 좌우정렬 신호 송신')
            line_number = 129

    return line_number


# -----------------------------------------------

def mode_ewsn(binary):
    ewsn = 129

    # 모폴로지 연산(열림연산) 후 컨투어
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    binary_ero = cv2.erode(binary, kernel)
    contours, _ = cv2.findContours(binary_ero, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 프레임 크기의 검은색 마스크 생성
    height, width = binary.shape[:2]
    mask_black = np.zeros((height, width, 1), dtype=np.uint8)

    # 컨투어된 영역중에서 제일 큰 부분만 선택 (배경 제거)
    max_contour = None
    max_area = -1

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            max_contour = contour

    # 방위 마스크 생성 후 다시 컨투어
    cv2.drawContours(mask_black, [max_contour], 0, 255, -1)
    contours, hierarchy = cv2.findContours(mask_black, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cv2.imshow("mask_black", mask_black)

    for contour in contours:
        # 각 컨투어에 근사 컨투어로 단순화
        approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
        # 꼭짓점의 개수
        vertices = len(approx)
        # 사각형으로 컨투어
        x, y, w, h = cv2.boundingRect(contour)

    if vertices >= 18:
        ewsn = 112
        print('남')
    elif vertices <= 12:
        ewsn = 111
        print('북')
    else:
        if abs(w - h) >= 60:
            ewsn = 113
            print('동')
        else:
            ewsn = 114
            print('서')

    terminate_time = timeit.default_timer()  # 종료 시간 체크
    print("%f초 걸렸습니다." % (terminate_time - start_time))
    print("방위 :", ewsn)
    return ewsn


def mode_arrow(binary):
    arrow = 129

    # 모폴로지 연산(흰색영역 확장) 후 컨투어
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    binary_dil = cv2.dilate(binary, kernel)
    contours, hierarchy = cv2.findContours(binary_dil, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # 프레임 크기의 검은색 마스크 생성
    height, width = binary.shape[:2]
    mask_black = np.zeros((height, width, 1), dtype=np.uint8)

    # 컨투어된 영역중에서 제일 큰 부분만 선택 (배경 제거)
    max_contour = None
    max_area = -1

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            max_contour = contour

    # 화살표 마스크 생성 후 다시 컨투어
    cv2.drawContours(mask_black, [max_contour], 0, 255, -1)
    contours, hierarchy = cv2.findContours(mask_black, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # 결함 찾기
    for cnt in contours:
        hull = cv2.convexHull(cnt, returnPoints=False)
        defects = cv2.convexityDefects(cnt, hull)

    left, right = 0, 0
    x, y = 10, 10

    # 결함의 주변 좌표가 컨투어 영역 안에 있는지 밖에 있는지 확인
    for i in range(defects.shape[0]):
        s, e, f, d = defects[i, 0]
        fx, fy = tuple(cnt[f][0])

        if d > 1000:
            cv2.circle(blur, (fx, fy), 5, (100, 0, 255), -1)

            if cv2.pointPolygonTest(contours[0], (fx + x, fy + y), False) == 1:
                right += 1
            if cv2.pointPolygonTest(contours[0], (fx + x, fy - y), False) == 1:
                right += 1
            if cv2.pointPolygonTest(contours[0], (fx - x, fy + y), False) == 1:
                left += 1
            if cv2.pointPolygonTest(contours[0], (fx - x, fy - y), False) == 1:
                left += 1

    if left < right:
        arrow = 113
        print('오른쪽')
    else:
        arrow = 114
        print('왼쪽')

    terminate_time = timeit.default_timer()  # 종료 시간 체크
    print("%f초 걸렸습니다." % (terminate_time - start_time))
    print("화살표방향 :", arrow)
    return arrow


def mode_alphaColor(blue_mask):
    color = 129

    cv2.imshow('blue_mask', blue_mask)
    pixels = cv2.countNonZero(blue_mask)

    if pixels > 500:
        color = 128
        print('blue')

    else:
        color = 130
        print('red')

    return color


def mode_abcd(binary_mask):
    abcd = 129

    contours, hierarchy = cv2.findContours(binary_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)  # 컨투어

    if len(contours) == 0:
        print("물체를 감지할 수 없습니다")
    else:
        contr = contours[0]
        x, y, w, h = cv2.boundingRect(contr)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
        frame2 = frame[y:y + h, x:x + w]

        cv2.drawContours(frame, contours, -1, (0, 255, 0), 4)

        blue_area = cv2.contourArea(contr, False)
        total_size = frame2.size
        per = blue_area / total_size

        if per < 0.145:
            abcd = 112
            print('c')
        elif per > 0.15 and per < 0.2:
            abcd = 113
            print('a')
        else:
            if len(contours) == 2:
                abcd = 111
                print('d')
            else:
                abcd = 114
                print('b')
    return abcd


def mode_areaColor(mask_green):
    areaColor = 129

    pixels = cv2.countNonZero(mask_green)

    if pixels > 10000:
        areaColor = 130  # 초록
        print('green')
    else:
        areaColor = 128
        print('black')

    terminate_time = timeit.default_timer()  # 종료 시간 체크
    print("%f초 걸렸습니다." % (terminate_time - start_time))
    print("영역색상 :", areaColor)
    return areaColor


def mode_milkSave(cambinary):
    mission = '128'
    contours, hierarchy = cv2.findContours(cam_binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)  # 컨투어
    if len(contours) == 0:
        print("물체를 감지할 수 없습니다")
    else:
        contour = contours[0]
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
        frame2 = frame[y:y + h, x:x + w]
        ret, mask_black = cv2.threshold(frame2, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        pixels = cv2.countNonZero(mask_black)
        frame2_size = frame2.size
        if (frame2_size - pixels) > 0:
            ratio = pixels / (frame2_size - pixels)
            if ratio > 1:
                return mission
            else:
                mission = '130'
                return mission
        else:
            mission = '129'
            return mission


def mode_milkEscape(cambinary):
    mission = '128'
    contours, hierarchy = cv2.findContours(cam_binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)  # 컨투어
    if len(contours) == 0:
        print("물체를 감지할 수 없습니다")
    else:
        contour = contours[0]
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
        frame2 = frame[y:y + h, x:x + w]
        ret, mask_black = cv2.threshold(frame2, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        pixels = cv2.countNonZero(mask_black)
        frame2_size = frame2.size
        if (frame2_size - pixels) > 0:
            ratio = pixels / (frame2_size - pixels)
            if ratio < 1:
                return mission
            else:
                mission = '130'
                return mission
        else:
            mission = '129'
        return mission


# def milk_check(cam_binary):

# return frame2


if __name__ == '__main__':
    BPS = 4800  # 4800,9600,14400, 19200,28800, 57600, 115200

    # ---------local Serial Port : ttyS0 --------
    # ---------USB Serial Port : ttyAMA0 --------

    serial_port = serial.Serial('/dev/ttyS0', BPS, timeout=0.01)
    serial_port.flush()  # serial cls

    # ---------------------------

    # serial_t = Thread(target=Receiving, args=(serial_port,))
    # serial_t.daemon = True
    # serial_t.start()
    # time.sleep(0.1)
    # ---------------------------

    # First -> Start Code Send
    TX_data_py2(serial_port, 128)

    # time.sleep(1)

    # -----  remocon 16 Code  Exit ------
    # while receiving_exit == 1:
    #    time.sleep(0.01)

    # ---------------------------
    # time.sleep(1)
    print("serial_port.readline = ", serial_port.readline())

    TX_data_py2(serial_port, 1)
    print("Return DATA: " + str(RX_data(serial_port)))
    print("-------------------------------------")

    # exit(1)

W_View_size = 640
H_View_size = 480
FPS = 90  # PI CAMERA: 320 x 240 = MAX 90

try:
    cap = cv2.VideoCapture(0)  # 카메라 켜기  # 카메라 캡쳐 (사진만 가져옴)

    cap.set(3, W_View_size)
    cap.set(4, H_View_size)
    cap.set(5, FPS)

except:
    print('cannot load camera!')

while True:
    start_time = timeit.default_timer()  # 시작 시간 체크
    ret, frame = cap.read()  # 무한루프를 돌려서 사진을 동영상으로 변경   # ret은 true/false

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # BGR을 HSV모드로 전환

    if ret:  # 사진 가져오는것을 성공할 경우
        cv2.imshow('Original', frame)

    else:
        print('cannot load camera!')
        break

    k = cv2.waitKey(25)
    if k == 27:
        break

    data = str(RX_data(serial_port))

    if data != '0':  # 수신된 data가 없으면 0이 리턴됨
        print("Return DATA: " + str(RX_data(serial_port)))
        print("-------------------------------------")
        print("-------------------------------------\n")

    if data == '201':  # 동서남북 검출
        print("Return DATA: " + str(RX_data(serial_port)))
        print("동서남북검출시작")
        ret, binary = cv2.threshold(blur, 90, 255, cv2.THRESH_BINARY_INV)  # 이진화
        res_ewsn = mode_ewsn(binary)

        if res_ewsn == 111 or res_ewsn == 112 or res_ewsn == 113 or res_ewsn == 114:
            print(res_ewsn)
            TX_data_py2(serial_port, res_ewsn)
            print("동서남북검출성공\n")


    elif data == '202':  # 화살표 검출
        print("Return DATA: " + str(RX_data(serial_port)))
        print("화살표검출시작")
        ret, binary = cv2.threshold(blur, 90, 255, cv2.THRESH_BINARY_INV)  # 이진화
        res_arrow = mode_arrow(binary)

        if res_arrow == 113 or res_arrow == 114:
            print(res_arrow)
            TX_data_py2(serial_port, res_arrow)
            print("화살표검출성공\n")



    elif data == '203':  # 알파벳색상 검출
        print("Return DATA: " + str(RX_data(serial_port)))
        print("알파벳색검출시작")
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        blur_mask = cv2.bitwise_and(blur, blur, mask=mask)
        res_alphacolor = mode_alphaColor(blur_mask)

        if res_alphacolor == 130 or res_alphacolor == 128 or res_alphacolor == 129:
            print(res_alphacolor)
            TX_data_py2(serial_port, res_alphacolor)
            print("알파벳색검출성공\n")


    elif data == '204':  # ABCD 검출
        print("Return DATA: " + str(RX_data(serial_port)))
        print('ABCD검출시작')

        if res_alphacolor == 128:
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
        elif res_alphacolor == 130:
            mask = cv2.inRange(hsv, lower_red, upper_red)

        blur_mask = cv2.bitwise_and(blur, blur, mask=mask)
        cv2.imshow("blur_mask", blur_mask)
        ret, binary_mask = cv2.threshold(blur_mask, 127, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)  # 이진화
        res_abcd = mode_abcd(binary_mask)

        if res_abcd == 111 or res_abcd == 112 or res_abcd == 113 or res_abcd == 114:
            print(res_abcd)
            TX_data_py2(serial_port, res_abcd)
            print("ABCD검출종료")


    elif data == '205':  # 영역색상 검출
        print("Return DATA: " + str(RX_data(serial_port)))
        print('영역색상 검출시작')
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        res_area = mode_areaColor(mask_green)

        if res_area == 130 or res_area == 128:
            print(res_area)
            TX_data_py2(serial_port, res_area)
            print("영역색검출종료")


    elif data == '206':  # 미션수행 확인
        print("Return DATA: " + str(RX_data(serial_port)))
        print('안전구역미션확인')

        if res_alphacolor == 128:
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
        elif res_alphacolor == 130:
            mask = cv2.inRange(hsv, lower_red, upper_red)

        result = cv2.bitwise_and(blur, blur, mask=mask)
        ret, cam_binary = cv2.threshold(result, 127, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)  # 이진화
        mission = mode_milkSave(cam_binary)

        if mission == 130:
            print(mission)
            TX_data_py2(serial_port, mission)
            print("미션완료\n")

    elif data == '216':  # 미션수행 확인
        print("Return DATA: " + str(RX_data(serial_port)))
        print('확진구역미션확인')

        if res_alphacolor == 128:
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
        elif res_alphacolor == 130:
            mask = cv2.inRange(hsv, lower_red, upper_red)

        result = cv2.bitwise_and(blur, blur, mask=mask)
        ret, cam_binary = cv2.threshold(result, 127, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)  # 이진화
        mission = mode_milkEscape(cam_binary)

        if mission == 130:
            print(mission)
            TX_data_py2(serial_port, mission)

            print("미션완료\n")

    elif data == '210':  # 라인트레이서
        print("Return DATA: " + str(RX_data(serial_port)))
        print('라인트레이서시작')

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)  # 노랑최소최대값을 이용해서 maskyellow값지정
        result = cv2.bitwise_and(blur, blur, mask=mask)
        res_line = linetracer(result)

        if res_line == 112 or res_line == 113 or res_line == 114 or res_line == 126 or res_line == 129:
            print(res_line)
            TX_data_py2(serial_port, res_line)
            print("라인검출성공\n")

cap.release()
cv2.destroyAllWindows()

terminate_time = timeit.default_timer()  # 종료 시간 체크
print("%f초 걸렸습니다." % (terminate_time - start_time))