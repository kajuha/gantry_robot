Status status
int32 position
float64 location
int32 speed
uint8 alarm # 서보알람
uint8 ready # 서보레디
uint8 inpos1 # 위치 도달 완료 1(0x2401, 0x2402참고)
uint8 inspd # 속도 도달 완료(0x2406참고)
uint8 org # 원점운전 완료
uint8 eos # 인덱스운전 완료