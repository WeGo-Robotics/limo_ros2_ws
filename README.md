# limo_ros2_ws

1.터미널에 다음과 같이 입력하여 코드를 다운로드 받습니다.

```
git clone https://github.com/WeGo-Robotics/limo_ros2_ws.git
```

2.다운로드 받은 ```limo_ws``` 폴더로 이동합니다.

```
cd limo_ros2_ws
```

3. 터미널에 다음과 같이 입력하여, 원클릭 셋팅 파일의 권한을 열어줍니다.

```
sudo chmod 777 permission.bash
```

4. 터미널에 다음(```password for $USER:```)과 같이 출력되면 패스워드를 입력합니다.
  
5. 터미널에 다음과 같이 입력하여 원클릭 셋팅 파일을 실행합니다.
```
./permission.bash
```

※원클릭 셋팅 파일(```permission.sh```)은 빌드(```catkin_make```) 및 사용하시는 쉘(bash or zsh)에 맞춰서 쉘의 환경설정 파일(```~/.bashrc``` or ```~/.zshrc```)에 현재 워크스페이스(```$Current_path```)의 경로를 설정(```source install/setup.bash``` or ```source install/setup.zsh```)합니다.

```
