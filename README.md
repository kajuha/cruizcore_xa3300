# CruizCore XA 3300 패키지 설치 및 사용방법

1. CruizCore-xa3300 패키지 다운로드
```
$ cd catkin_ws/src/
$ git clone https://github.com/Neoplanetz/KETI-ADCM-1B.git
$ cd KETI-ADCM-1B
$ mv cruizcore_xa3300/ ~/catkin_ws/src/
$ cd ..
$ rm -rf KETI-ADCM-1B
```

2. cruizcore-xa3300 패키지 catkin 빌드
```
$ cd ~/catkin_ws/
$ catkin_make
```

3. cruizcore_xa3300_driver 실행을 위한 권한 설정
```
$ cd ~/catkin_ws/devel/lib/cruizcore_xa3300_driver
$ sudo chown root:root cruizcore_xa3300_driver
$ sudo chmod a+rx cruizcore_xa3300_driver
$ sudo chmod u+s cruizcore_xa3300_driver
```

4. cruizcore_xa3300_driver 실행
```
$ roscore
$ sudo stty -F /dev/ttyUSB0 115200 raw
$ roslaunch cruizcore_xa3300_driver cruizcore_xa3300_driver.launch
```
