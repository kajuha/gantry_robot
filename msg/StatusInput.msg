uint8 p_ot # CCW 회전금지(0x2013참고)
uint8 n_ot # CW 회전금지(0x2013참고)
uint8 home # 원점센서
uint8 stop # 서보정지
uint8 pcon # P 제어동작(PI->P)
uint8 gain2 # 게인 1, 2 전환
uint8 p_cl # 정방향 토크제한(0x2110, 0x2111참고)
uint8 n_cl # 역방향 토크제한(0x2110, 0x2111참고)
uint8 mode # 운전모드 전환
uint8 reserved1
uint8 emg # 비상정지(0x2013참고)
uint8 a_rst # 알람리셋
uint8 sv_on # 서보 ON
uint8 spd1_lvsf1 # 다단속도 1(0x2312~0x2319참고), 진동제어필터 1(0x2515참고)
uint8 spd2_lvsf2 # 다단속도 2(0x2312~0x2319참고), 진동제어필터 2(0x2515참고)
uint8 spd3 # 다단속도 3(0x2312~0x2319참고)
uint8 start # 운전 개시
uint8 pause # 일시 정지
uint8 regt # 센서후 운전
uint8 hstart # 원점운전 개시
uint8 isel0 # 인덱스 번호 0
uint8 isel1 # 인덱스 번호 1
uint8 isel2 # 인덱스 번호 2
uint8 isel3 # 인덱스 번호 3
uint8 isel4 # 인덱스 번호 4
uint8 isel5 # 인덱스 번호 5
uint8 absrq # 절대위치데이터 요구
uint8 jstart # 조그운전(0x2300참고)
uint8 jdir # 조그 회전방향 선택
uint8 pclear # 입력펄스 클리어(0x3005참고)
uint8 aovr # 속도 오버라이드 선택
uint8 reserved2