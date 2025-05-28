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
sudo btmon
vim ~/ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim ~/ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/ble_agent.py
vim ~/ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim ~/ros2_kart_ws/src/app_interface_pkg/setup.py 
vim ~/ros2_kart_ws/src/app_interface_pkg/package.xml 
vim ~/ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/ble_agent.py
sudo btmon
vim ~/ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/ble_node.py 
cd ros2_kart_ws/
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node
pip install dbus-next
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
vim ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/ble_agent.py 
colcon build --packages-select app_interface_pkg
ls
rm -r build/ install/ log/
cd ros2_kart_ws/
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node
vim ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/ble_agent.py.py 
vim ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/ble_agent.py
cd ros2_kart_ws/
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node
vim /src/app_interface_pkg/app_interface_pkg/ble_agent.py
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py
sudo bluetooth
vim src/app_interface_pkg/app_interface_pkg/ble_node.py
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py
vim src/app_interface_pkg/app_interface_pkg/ble_node.py
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ls ~
ls ~ -al
ros2 run app_interface_pkg ble_node
vim src/app_interface_pkg/app_interface_pkg/ble_node.py
sudo systemctl daemon-reload 
sudo systemctl restart bluetooth
ros2 run app_interface_pkg ble_node
vim src/app_interface_pkg/app_interface_pkg/ble_node.py
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node
ls /var/lib/bluetooth/
sudo ls /var/lib/bluetooth/
sudo ls /var/lib/bluetooth/ -al
sudo ls /var/lib/bluetooth/D8:3A:DD:F2:4C:84
sudo bluetoothctl 
ros2 run app_interface_pkg ble_node
cd ..
pip freeze > requirements.txt 
cat requirements.txt | grep gatt
git add .
git commit -m "Add: ble node"
git push -u origin main
echo ".git-credentials" >> .gitignore 
git rm --cached .git-credentials 
git add .gitignore 
git commit -m "Remove: .git-credentials and ignore it"
git push -u origin main
git log
git push --force origin main
git filter-branch --force   --index-filter "git rm --cached --ignore-unmatch .git-credentials"   --prune-empty   --tag-name-filter cat   -- --all
git push -u origin main
git log
git filter-branch --force   --index-filter "git rm --cached --ignore-unmatch .git-credentials"   --prune-empty   --tag-name-filter cat   -- --all
rm -rf .git/filter-branch/
git for-each-ref --format="%(refname)" refs/original/ |   xargs -n 1 git update-ref -d
rm -rf .git/filter-branch/git for-each-ref --format="%(refname)" refs/original/ |   xargs -n 1 git update-ref -d
rm -d .git/filter-branch/git for-each-ref --format="%(refname)" refs/original/ |   xargs -n 1 git update-ref -d
rm -rf .git/filter-branch/
git for-each-ref --format="%(refname)" refs/original/ |   xargs -n 1 git update-ref -d
git reflog expire --expire=now --all
git gc --prune=now --aggressive
git push -u origin main
git log
git reset --hard f3285e7bd4b9ca4cd796e2d8aa284fdca85ed4c5
git log
cat .gitignore 
git revert f3285e7bd4b9ca4cd796e2d8aa284fdca85ed4c5
git log
ls
cd ros2_kart_ws/src/
ls
git reset --hard f3285e7bd4b9ca4cd796e2d8aa284fdca85ed4c5
git log
cd ros2_kart_ws/src/
ls
git reset --soft HEAD~1
git log
ls
cd ~
echo ".git-credentials" >> .gitignore 
ls
git add .
git commit -m "Add app_interface_pkg"
git push -u origin main
git log
git push -u origin main
sudo poweroff 
ros2 run app_interface_pkg ble_node 
ls
cd ros2_kart_ws/
ls
ls src/
ls src/interfaces/msg/
ls
vim src/decision_pkg/decision_pkg/decision_node.py
                                                                                ros2 run app_interface_pkg ble_node 
vim ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/ble_node.py 
ros2 run app_interface_pkg ble_node 
cd ros2_kart_ws/
colcon build --packages-select app_interface
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface
source install/setup.bash 
ls src/
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
cd ..
ls
vim ros2_humble_venv/lib/python3.11/site-packages/bluezero/peripheral.py 
colcon build --packages-select app_interface_pkg
ls
rm -r build/ install/ log/
ls
cd ros2_kart_ws/
colcon build --packages-select app_interface_pkg
vim ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/ble_node.py 
cd ros2_kart_ws/
ls
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
sudo poweroff 
vim ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/ble_node.py 
cd ros2_kart_ws/
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
source install/setup.bash 
ros2 run app_interface_pkg ble_node 
cd src/
cd ..
ls
vim src/sensor_pkg/sensor_pkg/illuminance_node.py
sudo poweroff 
btmon
sudo btmon
vim ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/ble_node.py 
sudo btmon
vim ros2_kart_ws/src/app_interface_pkg/app_interface_pkg/ble_node.py 
clear
vim ros2_kart_ws/src/sensor_pkg/sensor_pkg/imu_node.py 
ls
git add .
git status
git commit -m "Fix: ble write_callback"
git branch dev
git branch list
git branch -l
git branch -rm list
git branch -d list
git checkout dev 
git branch
git push -u dev dev
git push -u origin dev
git branch main 
git checkout main 
git push -u origin dev
ls
cat token_code.txt 
git push -u origin dev
echo "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" >> token_code.txt 
cat token_code.txt 
echo "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" > token_code.txt 
cat token_code.txt 
git push -u origin dev
git push origin main
ls ros2_kart_ws/src/
ls ros2_kart_ws/src/decision_pkg/decision_pkg/
vim ros2_kart_ws/src/decision_pkg/decision_pkg/decision_node.py 
git checkout dev 
vim ros2_kart_ws/src/decision_pkg/decision_pkg/decision_node.py 
vim ros2_kart_ws/src/sensor_pkg/sensor_pkg/illuminance_node.py 
vim ros2_kart_ws/src/sensor_pkg/package.xml 
vim ros2_kart_ws/src/sensor_pkg/setup.py 
cd ros2_kart_ws/
colcon build --packages-select sensor_pkg
source install/setup.bash 
ros2 run sensor_pkg illuminance_pub 
vim src/sensor_pkg/sensor_pkg/illuminance_node.py 
colcon build --packages-select sensor_pkg
source install/setup.bash 
ros2 run sensor_pkg illuminance_pub 
ls /dev/i2c-*
cat /boot/config.txt 
cat /boot/firmware/config.txt 
sudo vim /boot/firmware/config.txt 
ls /dev/i2c-*
sudo reboot 
ls /dev/i2c-*
cd ros2_kart_ws/
vim src/sensor_pkg/sensor_pkg/illuminance_node.py 
colcon build --packages-select sensor_pkg
source install/setup.bash 
ros2 run sensor_pkg illuminance_pub 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
ls
ls src/
cd src/
ros2 pkg create --build-type ament_python --license Apache-2.0 actuator_pkg
