ls
ifconfig 
poweroff 
sudo poweroff 
source ros2_humble_venv/bin/activate
cd ros2_humble_ws/
pip install lark-parser
colcon build
source ros2_humble_venv/
source ros2_humble_venv/bin/activate
cd ros2_humble_ws/
colcon build
ls
source ros2_humble_venv/
source ros2_humble_venv/bin/activate
cd ros2_humble_ws/
colcon build
source ros2_humble_venv/bin/activate
cd ros2_humble_ws/
pip install PyQt5 sip
sudo apt install python3-pyqt5 python3-sip
colcon build
sudo apt install vim
vim ~/ros2_humble_venv/pyvenv.cfg 
deactive
cd ..
deactive
deactivate
source ros2_humble_venv/bin/activate
cd ros2_humble_ws/
colcon build
ls
ps -ef
cd ros2_humble_ws/
ls install/
ls
ls -al
vim .bashrc 
source ros2_humble_venv/bin/activate
ls /opt
cd ros2_humble_ws/
ls
ls src/
cd ..
ls
mkdir ros2_kart_ws
ls
source ros2_humble_ws/install/setup.bash 
cd ros2_kart_ws/
ls
colcon build
ls
source install/setup.bash 
vim ~/.bashrc 
reboot 
sudo reboot 
ls
cd ros2_kart_ws/
ls
mkdir src
cd src/
ros2 pkg create --build-type ament_python --license Apache-2.0 kart
ls
cd kart/
ls
cd kart/
ls
cd ..
mkdir src
cd src/
ls
vim bleNode.py
cd ..
ls
cd ..
colcon build --packages-select kart
source install/setup.bash 
cd k
cd src/
ls
cd kart/
ls
ls resource/
vim package.xml 
vim setup.py 
cd ../..
ls
colcon build --packages-select kart
ros2 run kart ble_peripheral_node 
cd src/kart/
ls
vim setup.py 
cd src/
ls
cd ..
vim setup.py 
cd ../..
colcon build --packages-select kart
source install/setup.bash 
ros2 run kart ble_peripheral_node 
cd src/kart/
ls
mv src/bleNode.py kart/
rm src/
ls kart/
rm -r src/
cd ../..
colcon build --packages-select kart
source install/setup.bash 
ros2 run kart ble_peripheral_node 
pip install bluezero
sudo apt update
vim src/kart/kart/bleNode.py 
vim src/kart/package.xml 
vim src/kart/setup.py 
pip3 install gatt
colcon build --packages-select kart
source install/setup.bash 
ros2 run kart ble_peripheral_node 
sudo apt install -y   python3-dbus   python3-dbus.mainloop.glib   libdbus-glib-1-dev
sudo 
sudo apt install -y   python3-dbus  
vim src/kart/package.xml 
colcon build --packages-select kart
source install/setup.bash 
ros2 run kart ble_peripheral_node 
vim src/kart/kart/bleNode.py 
colcon build --packages-select kart
cd ros2_kart_ws/
source install/setup.bash 
ros2 run kart ble_peripheral_node 
vim src/kart/kart/bleNode.py 
colcon build --packages-select kart
source install/setup.bash 
ros2 run kart ble_peripheral_node 
vim src/kart/kart/bleNode.py 
colcon build --packages-select kart
source install/setup.bash 
ros2 run kart ble_peripheral_node 
vim src/kart/kart/bleNode.py 
colcon build --packages-select kart
source install/setup.bash 
ros2 run kart ble_peripheral_node 
vim src/kart/kart/bleNode.py 
colcon build --packages-select kart
source install/setup.bash 
ros2 run kart ble_peripheral_node 
vim ros2_kart_ws/src/kart/kart/bleNode.py 
ros2 run kart ble_peripheral_node 
vim ros2_kart_ws/src/kart/kart/bleNode.py 
cd ros2_kart_ws/
colcon build --packages-select kart
source install/setup.bash 
ros2 run kart ble_peripheral_node 
vim ros2_kart_ws/src/kart/kart/bleNode.py 
ls
vim src/kart/kart/bleNode.py 
colcon build --packages-select kart
source install/setup.bash 
ros2 run kart ble_peripheral_node 
vim src/kart/kart/bleNode.py 
colcon build --packages-select kart
source install/setup.bash 
ros2 run kart ble_peripheral_node 
vim src/kart/kart/bleNode.py 
colcon build --packages-select kart
source install/setup.bash 
ros2 run kart ble_peripheral_node 
sudo poweroff 
ls
cd ros2_kart_ws/
cd src/
ls
mkdir vision_pkg sensor_pkg decision_pkg
ls
rm -r decision_pkg/ sensor_pkg/ vision_pkg/
ros2 pkg create --build-type ament_python --license Apache-2.0 vision_pkg
ros2 pkg create --build-type ament_cmake --license Apache-2.0 sensor_pkg decision_pkg
ros2 pkg create --build-type ament_cmake --license Apache-2.0 sensor_pkg
ros2 pkg create --build-type ament_cmake --license Apache-2.0 decision_pkg
ls
rm -r decision_pkg/ sensor_pkg/
ros2 pkg create --build-type ament_python --license Apache-2.0 sensor_pkg
ros2 pkg create --build-type ament_python --license Apache-2.0 decision_pkg
ls
cd sensor_pkg/
ls
cd sensor_pkg/
ls
vim imu_node.py
ls
cd ..
mkdir msg
ls
cd msg/
cd ..
rm -r msg/
cd ..
ls
ros2 pkg create --build-type ament_cmake --license Apache-2.0 interfaces
mkdir interfaces/msg 
cd interfaces/
ls
cd msg/
cd ros2_kart_ws/
cd src/sensor_pkg/sensor_pkg/
ls
pip install smbus2
sudo raspi-config 
ls
vim imu_node.py
cd ~/ros2_kart_ws/src/interfaces/
ls
cd msg/
ls
vim Imu.msg
cd ~/ros2_kart_ws/src/sensor_pkg/sensor_pkg/
vim imu_node.py 
cd ~/ros2_kart_ws/src/interfaces/
ls
vim CM
vim CMakeLists.txt 
vim package.xml 
cd ~/ros2_kart_ws/
colcon build
sudo reboot 
cd ros2_kart_ws/
vim src/interfaces/CMakeLists.txt 
rm src/interfaces/.CMakeLists.txt.swp 
vim src/interfaces/package.xml 
colcon build --packages-select interfaces
ros2 run sensor_pkg imu_pub 
source install/setup.bash 
ros2 run sensor_pkg imu_pub 
lsmod | grep i2c
ls -l /dev/i2c-1 
ros2 run sensor_pkg imu_pub 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
colcon build --packages-select sensor_pkg
source install/setup.bash 
ros2 run sensor_pkg imu_pub 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
colcon build --packages-select sensor_pkg
source install/setup.bash 
ros2 run sensor_pkg imu_pub 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
colcon build --packages-select sensor_pkg
source install/setup.bash 
ros2 run sensor_pkg imu_pub 
cd ..
git remote add origin https://github.com/KR-D2Bro/Vision_Walker_Helper_HW.git
cd ros2_kart_ws/
git remote add origin https://github.com/KR-D2Bro/Vision_Walker_Helper_HW.git
cd ..
ls
echo "# Vision_Walker_Helper_HW" >> README.md
ls
git init
ls
git add README.md
git commit -m "first commit"
git config --global user.email "32204045@dankook.ac.kr"
git config --global user.name "dongjae"
git lis
git list
git --help
git log
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/KR-D2Bro/Vision_Walker_Helper_HW.git
git push -u origin main
ls
git remote add origin https://github.com/KR-D2Bro/Vision_Walker_Helper_HW.git
git push -u origin main
git remote set-url origin https://github.com/KR-D2Bro/Vision_Walker_Helper_HW.git
git push -u origin main
git remote set-url origin https://github.com/KR-D2Bro/Vision_Walker_Helper_HW.git
git push -u origin main
ls
vim .gitignore
vim token_code.txt
vim .gitignore
git add .
ls
pip freeze > requirements.txt
ls
cat requirements.txt 
ls
vim .gitignore 
cd ros2_humble_venv/
ls
cd ..
vim .gitignore 
git add .
vim .gitignore 
git add .
vim .gitignore 
git push -u origin main
git commit -m "add imu_pub"
git log
git push -u origin main
ls
git add .
git commit -m "remove ros2_humble_ws"
git push -u origin main
git rm -r --cached ros2_humble_ws/
ls
cd ros2_humble_ws/
ls
cd ..
git commit -m "Remove ROS2 build artifacts from repo"
git push -u origin main
sudo apt update
sudo apt install git-credential-manager-core
git config --global credential.helper store
git push -u origin main
ls
cd ros2_kart_ws/
ls
sudo poweroff 
ros2 topic list
ros2 topic echo imu/data
vim ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/ble_node.py 
cd ros2_kart_ws/
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/peripheral.py 
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/peripheral.py 
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node
vim ros2_humble_venv/lib/python3.11/site-packages/
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/ 
cd ros2_kart_ws/
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
sudo bluetoothctl
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/ble_node.py 
0;44;57M0;44;57mclear
clear
vim ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/ble_node.py 
rm ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/.ble_node.py.swp 
vim ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/ble_node.py 
colcon build --packages-select app_interface_pkg
ls
rm -r build/ install/ log/
cd ros2_kart_ws/
colcon build --packages-select app_interface_pkg
