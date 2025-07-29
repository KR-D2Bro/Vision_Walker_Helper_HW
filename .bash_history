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
source ros2_humble_venv/
source ros2_humble_venv/bin/activate
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
sudo poweroff 
ls
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
rm src/app_interface_pkg/app_interface_pkg/.ble_node.py.swp 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
cp src/app_interface_pkg/app_interface_pkg/ble_node.py src/app_interface_pkg/app_interface_pkg/ble_periph.py
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py
clear
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
rm src/app_interface_pkg/app_interface_pkg/.ble_periph.py.swp 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
cd ros2_kart_ws/
rm src/app_interface_pkg/app_interface_pkg/.ble_node.py.swp 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py
clear
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
sudo poweroff 
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
rm src/app_interface_pkg/app_interface_pkg/.ble_periph.py.swp 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
rm src/app_interface_pkg/app_interface_pkg/.ble_periph.py.swp 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/package.xml 
vim src/app_interface_pkg/setup.py 
colcon build
vim src/interfaces/msg/Config.msg 
colcon build
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
colcon build
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
colcon build
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
ls
cd ros2_kart_ws/
rm src/app_interface_pkg/app_interface_pkg/.ble_periph.py.swp 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/interfaces/msg/
vim src/interfaces/msg/config.msg
vim src/interfaces/msg/Imu.msg 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py
vim src/interfaces/msg/config.msg
vim src/interfaces/CMakeLists.txt 
mv src/interfaces/msg/config.msg src/interfaces/msg/Config.msg
vim src/interfaces/package.xml 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
rm src/app_interface_pkg/app_interface_pkg/.ble_node.py.swp 
ls
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
rm src/app_interface_pkg/app_interface_pkg/.ble_node.py.swp 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
rm src/app_interface_pkg/app_interface_pkg/.ble_periph.py.swp 
colcon build
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
colcon build
ros2 run app_interface_pkg ble_node 
cd ..
ls
vim config.conf
ros2 run app_interface_pkg ble_node 
vim config.conf 
ros2 run app_interface_pkg ble_node 
vim config.conf 
ros2 run app_interface_pkg ble_node 
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
ros2 run app_interface_pkg ble_node 
colcon build --select-packages app_interface_pkg
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
cd ros2_kart_ws/
ls
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
colcon build
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
colcon build
sudo poweroff 
cd ros2_
cd ros2_kart_ws/
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
ros2 run app_interface_pkg ble_node 
sudo poweroff 
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
ls ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/peripheral.py 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/advertisement.py 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/GATT.py 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/localGATT.py 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/peripheral.py 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/localGATT.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/localGATT.py 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/peripheral.py 
sudo poweroff 
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/localGATT.py 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/peripheral.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/localGATT.py 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/peripheral.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
sudo poweroff
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/peripheral.py 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/peripheral.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/peripheral.py 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/localGATT.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/peripheral.py 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/localGATT.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
sudo bluetoothctl 
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
sudo poweroff 
btmon
sudo btmon
clear
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
sudo poweroff 
sudo btmon
clear
sudo btmon
clear
sudo btmon
clear
sudo btmon
clear
sudo btmon
clear
sudo btmon
clear
sudo btmon
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim ~/secretKey.txt 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
cd ..
vim secretKey.txt
cd ros2_kart_ws/
colcon build --packages-select app_interface_pkg
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
ros2 run app_interface_pkg ble_node 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim ~/secretKey.txt 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
ros2 run app_interface_pkg ble_node 
bluetoothctl 
ros2 run app_interface_pkg ble_node 
bluetoothctl 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
bluetoothctl 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
bluetoothctl 
ls /var/lib/bluetooth/
sudo ls /var/lib/bluetooth/
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
colcon build --packages-select app_interface_pkg
bluetoothctl 
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
ls ~/ros2_humble_venv/lib/python3.11/site-packages/dbus_next/errors.py 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/dbus_next/errors.py 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
ls src/app_interface_pkg/app_interface_pkg/
vim ~/ros2_humble_venv/lib/python3.11/site-packages/dbus_next/errors.py 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
colcon build --packages-select app_interface_pkg
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim ~/secretKey.txt 
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim ~/secretKey.txt 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
ls
cd ros2_kart_ws/
ls
vim src/sensor_pkg/sensor_pkg/battery_node.py
clear
cd ros2_kart_ws/
vim src/sensor_pkg/sensor_pkg/battery_node.py 
rm src/sensor_pkg/sensor_pkg/.battery_node.py.swo
rm src/sensor_pkg/sensor_pkg/.battery_node.py.swp 
vim src/sensor_pkg/sensor_pkg/battery_node.py 
rm src/sensor_pkg/sensor_pkg/.imu_node.py.swp 
vim src/sensor_pkg/sensor_pkg/battery_node.py 
vim src/interfaces/msg/Battery_state.msg
vim src/interfaces/package.xml 
vim src/interfaces/CMakeLists.txt 
vim src/sensor_pkg/sensor_pkg/battery_node.py 
colcon build
vim src/sensor_pkg/sensor_pkg/battery_node.py 
colcon build
source install/setup.bash 
ros2 interface show interfaces/msg/BatteryState
vim src/sensor_pkg/setup.py 
vim src/sensor_pkg/package.xml 
vim src/sensor_pkg/sensor_pkg/battery_node.py 
colcon build --packages-select sensor_pkg
ros2 topic echo /battry_state/data
ros2 run sensor_pkg battery_pub 
vim src/sensor_pkg/sensor_pkg/battery_node.py 
colcon build --packages-select sensor_pkg
ros2 run sensor_pkg battery_pub 
source install/setup.bash
ros2 run sensor_pkg battery_pub 
vim src/sensor_pkg/sensor_pkg/battery_node.py 
colcon build --packages-select sensor_pkg
ros2 run sensor_pkg battery_pub 
vim src/sensor_pkg/sensor_pkg/battery_node.py 
colcon build --packages-select sensor_pkg
ros2 run sensor_pkg battery_pub 
vim src/sensor_pkg/sensor_pkg/battery_node.py 
colcon build --packages-select sensor_pkg
ros2 run sensor_pkg battery_pub 
sudo poweroff 
clear
cd ros2_kart_ws/
vim src/sensor_pkg/sensor_pkg/imu_node.py 
vim src/interfaces/msg/Imu.msg 
mv src/interfaces/msg/Battery_state.msg src/interfaces/msg/BatteryState.msg
vim src/interfaces/CMakeLists.txt 
ls src/sensor_pkg/sensor_pkg/
vim src/sensor_pkg/sensor_pkg/imu_node.py 
vim src/sensor_pkg/sensor_pkg/i
vim src/sensor_pkg/sensor_pkg/illuminance_node.py 
source install/setup.bash
ros2 topic echo /battery_state/data
ls
vim ros2_kart_ws/src/sensor_pkg/sensor_pkg/imu_node.py 
vim ros2_kart_ws/src/sensor_pkg/sensor_pkg/illuminance_node.py 
cd ros2_kart_ws/
vim src/sensor_pkg/sensor_pkg/battery_node.py
rm src/sensor_pkg/sensor_pkg/.battery_node.py.swp 
vim src/sensor_pkg/sensor_pkg/battery_node.py
clear
vim ros2_kart_ws/src/sensor_pkg/sensor_pkg/illuminance_node.py 
rm ros2_kart_ws/src/sensor_pkg/sensor_pkg/.illuminance_node.py.swp 
vim ros2_kart_ws/src/sensor_pkg/sensor_pkg/illuminance_node.py 
vim ros2_kart_ws/src/sensor_pkg/sensor_pkg/imu_node.py 
vim ros2_kart_ws/src/sensor_pkg/sensor_pkg/illuminance_node.py 
vim ros2_kart_ws/src/sensor_pkg/sensor_pkg/imu_node.py
vim ros2_kart_ws/src/sensor_pkg/sensor_pkg/illuminance_node.py 
cd ros2_kart_ws/
vim src/sensor_pkg/sensor_pkg/battery_node.py 
rm src/sensor_pkg/sensor_pkg/.battery_node.py.swp 
vim src/sensor_pkg/sensor_pkg/battery_node.py 
cd ros2_kart_ws/
vim src/sensor_pkg/sensor_pkg/battery_node.py 
cd ros2_kart_ws/
vim src/sensor_pkg/sensor_pkg/illuminance_node.py 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
vim src/sensor_pkg/sensor_pkg/illuminance_node.py 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
ls /dev/i2c-*
vim src/sensor_pkg/sensor_pkg/battery_node.py
cd ros2_kart_ws/
vim src/sensor_pkg/sensor_pkg/battery_node.py
sudo poweroff 
cd ros2_kart_ws/
vim src/sensor_pkg/sensor_pkg/battery_node.py 
sudo poweroff 
ros2 run sensor_pkg battery_pub 
sudo poweroff 
ros2 topic info /battery_state
ros2 topic info /battery_state/data
ros2 topic echo /battery_state/data
cd ros2_kart_ws/
mkdir src/interfaces/srv
vim src/interfaces/srv/LightCommand.srv
ls
vim src/actuator_pkg/actuator_pkg/light_service.py
vim src/actuator_pkg/setup.py 
vim src/interfaces/CMakeLists.txt 
vim src/interfaces/package.xml 
colcon build
ls
ros2 run actuator_pkg light_srv 
vim src/actuator_pkg/actuator_pkg/light_service.py
pip3 list | grep RPi
pip3 list
which pip3
pip3 install RPi.GPIO
vim src/actuator_pkg/actuator_pkg/light_service.py
vim src/actuator_pkg/setup.py 
vim src/actuator_pkg/package.xml 
colcon build
vim src/decision_pkg/decision_pkg/decision_node.py 
vim src/decision_pkg/package.xml 
colcon build
source install/setup.bash 
vim src/decision_pkg/setup.py 
colcon build
source install/setup.bash 
ros2 run decision_pkg decision_node 
vim src/decision_pkg/decision_pkg/decision_node.py 
colcon build
ros2 run decision_pkg decision_node 
ros2 run actuator_pkg light_srv 
ros2 run decision_pkg decision_node 
vim src/decision_pkg/decision_pkg/decision_node.py 
ros2 run decision_pkg decision_node 
vim src/decision_pkg/decision_pkg/decision_node.py 
ros2 run sensor_pkg battery_pub 
sudo poweroff 
ros2 run sensor_pkg battery_pub 
cd ros2_kart_ws/
vim src/sensor_pkg/setup.py 
ros2 run sensor_pkg battery_pub 
source install/setup.bash 
ros2 run actuator_pkg light_srv 
ros2 run sensor_pkg battery_pub 
vim src/sensor_pkg/sensor_pkg/battery_node.py 
colcon build --packages-select 
ros2 run sensor_pkg battery_pub 
vim src/sensor_pkg/sensor_pkg/battery_node.py 
vim src/actuator_pkg/actuator_pkg/buzzer_service.py
vim src/actuator_pkg/actuator_pkg/light_service.py 
vim src/actuator_pkg/actuator_pkg/buzzer_service.py
vim src/decision_pkg/decision_pkg/decision_node.py 
vim src/interfaces/srv/
vim src/interfaces/srv/BuzzerCommand.srv
vim src/decision_pkg/setup.py 
vim src/decision_pkg/package.xml 
vim src/actuator_pkg/setup.py 
vim src/actuator_pkg/package.xml 
colcon build
ros2 run decision_pkg decision_node 
vim src/interfaces/CMakeLists.txt 
colcon build
ros2 run decision_pkg decision_node 
source install/setup.bash 
ros2 run decision_pkg decision_node 
vim src/decision_pkg/decision_pkg/decision_node.py 
colcon build --packages-select decision_pkg
ros2 run actuator_pkg light_srv 
colcon build --packages-select decision_pkg
ros2 run actuator_pkg light_srv 
ls
vim yolo_test.py
./ros2_humble_venv/bin/python3 yolo_test.py 
ls /dev/video*
sudo raspi-config 
v4l2-ctl --list-devices
sudo modprobe bcm2835-v4l2
echo "bcm2835-v4l2" | sudo tee -a /etc/modules
ls /dev/video*
./ros2_humble_venv/bin/python3 yolo_test.py 
libcamera-hello 
dmesg | grep -i video
v4l2-ctl --device=/dev/video0 --all
sudo raspi-config 
ls
vim yolo_test.py 
./ros2_humble_venv/bin/python3 yolo_test.py 
vim yolo_test.py 
./ros2_humble_venv/bin/python3 -c "import cv2; print(cv2.__version__)"
vim yolo_test.py 
./ros2_humble_venv/bin/python3 yolo_test.py 
./ros2_humble_venv/bin/python3 -c "import cv2; print(cv2.__version__)"
which picamera2
vim yolo_test.py 
rm .yolo_test.py.swp 
vim yolo_test.py 
./ros2_humble_venv/bin/python3 yolo_test.py 
sudo apt update
sudo apt install -y python3-picamera2
./ros2_humble_venv/bin/python3 yolo_test.py 
vim yolo_test.py 
./ros2_humble_venv/bin/python3 yolo_test.py 
vim yolo_test.py 
./ros2_humble_venv/bin/python3 yolo_test.py 
vim yolo_test.py 
./ros2_humble_venv/bin/python3 yolo_test.py 
sudo apt install xserver-xorg raspberrypi-ui-mods
sudo reboot 
sudo raspi-config 
sudo reboot 
sudo raspi-config 
ls
vim yolo_test.py 
cd ros2_kart_ws/
colcon build
colcon build --packages-select vision_pkg
clear
cd ros2_kart_ws/src/vision_pkg/vision_pkg/
vim yolo_detector.py 
rm .yolo_detector.py.swo 
vim yolo_detector.py 
vim yolo_detector.py
sudo poweroff 
clear
ros2 run vision_pkg vision_node 
cd ros2_kart_ws/
vim src/vision_pkg/setup.py 
vim src/vision_pkg/vision_pkg/object_detection_node.py 
colcon build
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/object_detection_node.py 
vim src/vision_pkg/setup.py 
vim src/vision_pkg/vision_pkg/object_detection_node.py 
ros2 run vision_pkg vision_node 
colcon build --packages-select vision_pkg
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/object_detection_node.py 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
cd ros2_kart_ws/src/vision_pkg/vision_pkg/
ls
vim object_detection_node.py 
vim yolo_detector.py 
vim object_detection_node.py 
vim yolo_detector.py 
cd yolov5/
ls
cd utils/
ls
vim plots.py 
cd ../..
ls
vim yolo_detector.py 
cd yolov5/utils/
ls
vim general.py 
cd ../..
vim yolo_detector.py 
ros2 run vision_pkg vision_node 
cd ros2_kart_ws/
vim src/vision_pkg/vision_pkg/yolo_detector.py 
source install/setup.bash 
ros2 run vision_pkg vision_node 
colcon build --packages-select vision_pkg
ros2 run vision_pkg vision_node 
vim ~/yolo_test.py 
colcon build --packages-select vision_pkg
ros2 run vision_pkg vision_node 
vim ros2_kart_ws/src/vision_pkg/vision_pkg/yolo_detector.py 
ros2 run vision_pkg vision_node 
ls
cd ros2_kart_ws/
ls
vim src/actuator_pkg/actuator_pkg/buzzer_service.py 
clear
cd ros2_kart_ws/
vim src/actuator_pkg/actuator_pkg/buzzer_service.py 
vim src/interfaces/srv/BuzzerCommand.srv 
vim src/actuator_pkg/actuator_pkg/buzzer_service.py 
ros2 run actuator_pkg buzzer_srv 
vim src/actuator_pkg/setup.py 
vim src/actuator_pkg/package.xml 
ros2 run actuator_pkg light_srv 
vim src/actuator_pkg/actuator_pkg/buzzer_service.py 
ros2 run actuator_pkg buzzer_srv 
vim src/actuator_pkg/actuator_pkg/buzzer_service.py 
ros2 run actuator_pkg buzzer_srv 
clear
cd ros2_kart_ws/
vim src/actuator_pkg/actuator_pkg/buzzer_service.py 
rm src/actuator_pkg/actuator_pkg/.buzzer_service.py.swp 
vim src/actuator_pkg/actuator_pkg/buzzer_service.py 
vim src/decision_pkg/decision_pkg/decision_node.py 
colcon build
vim src/decision_pkg/decision_pkg/decision_node.py 
colcon build
vim src/decision_pkg/decision_pkg/decision_node.py 
colcon build
ros2 run decision_pkg decision_node 
colcon build --packages-select actuator_pkg
ros2 run decision_pkg decision_node 
vim src/decision_pkg/decision_pkg/decision_node.py 
colcon build --packages-select actuator_pkg
colcon build --packages-select decision_pkg
ros2 run decision_pkg decision_node 
vim src/decision_pkg/decision_pkg/decision_node.py 
colcon build --packages-select decision_pkg
ros2 run decision_pkg decision_node 
clear
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
colcon build --packages-select decision_pkg
ros2 run decision_pkg decision_node 
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
colcon build
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
rm -r src/kart/
cd src/
ros2 pkg create --build-type ament_python launch_pkg
ls
cd launch_pkg/
ls
mkdir launch
cd launch
launch.py
vim launch.py
vim ../setup.py 
cd ..
ls
vim package.xml 
vim setup.py 
vim launch/launch.py 
vim setup.py 
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
ros2 topic pub --once /battery_state/data interfaces/msg/BatteryState "data: 42"
ros2 topic pub --once /battery_state/data interfaces/msg/BatteryState "percentage: 42"
ls
ls src/sensor_pkg/sensor_pkg/
vim src/sensor_pkg/setup.py 
vim src/actuator_pkg/setup.py 
vim src/app_interface_pkg/
vim src/app_interface_pkg/setup.py 
vim src/decision_pkg/setup.py 
vim src/vision_pkg/setup.py 
colcon build
ros2 launch launch_pkg launch.py
source install/setup.bash 
ros2 launch launch_pkg launch.py
sudo poweroff 
ls
rm yolo_test.py 
rm token_code.txt 
cd ros2_kart_ws/
ls
vim src/sensor_pkg/sensor_pkg/imu_node.py 
vim /boot/config.txt 
vim /boot/firmware/config.txt 
sudo vim /boot/firmware/config.txt 
ls /dev/i2c-*
sudo vim /boot/firmware/config.txt 
sudo reboot 
ls
cd ros2_kart_ws/
vim src/sensor_pkg/sensor_pkg/battery_node.py 
ros2 run vision_pkg vision_node 
cd ros2_kart_ws/
source install/setup.sh 
ros2 run vision_pkg vision_node 
ros2 run app_interface_pkg ble_node 
cd ros2_kart_ws/
vim src/sensor_pkg/sensor_pkg/battery_node.py 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
vim src/sensor_pkg/setup.py 
vim src/sensor_pkg/package.xml 
colcon build --packages-select sensor_pkg
vim src/sensor_pkg/package.xml 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
colcon build --packages-select sensor_pkg
ros2 run sensor_pkg imu_pub 
ls /dev/i2c-*
vim src/sensor_pkg/sensor_pkg/imu_node.py 
colcon build --packages-select sensor_pkg
vim src/sensor_pkg/sensor_pkg/imu_node.py 
ros2 run sensor_pkg imu_pub 
i2cdetect -y 1
i2cdetect -y 3
ros2 run sensor_pkg imu_pub 
i2cdetect -y 3
ros2 run sensor_pkg imu_pub 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
colcon build --packages-select sensor_pkg
ros2 run sensor_pkg imu_pub 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
colcon build --packages-select sensor_pkg
ros2 run sensor_pkg imu_pub 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
i2cdetect -y 0
ls -l /dev/i2c-*
sudo i2cdetect -y 0
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/sensor_pkg/sensor_pkg/battery_node.py 
ros2 topic --once interfaces/msg /battery_state/data "percentage:30"
ros2 topic --once 
ros2 topic pub --once /battery_state/data interfaces/msg/BatteryState "percentage: 42"
vim src/sensor_pkg/sensor_pkg/imu_node.py 
sudo i2cdetect -y 3
ros2 topic pub --once /battery_state/data interfaces/msg/BatteryState "percentage: 42"
sudo i2cdetect -y 3
sudo i2cdetect -y 1
ros2 topic pub --once /battery_state/data interfaces/msg/BatteryState "percentage: 42"
vim src/sensor_pkg/sensor_pkg/battery_node.py 
rm src/sensor_pkg/sensor_pkg/.battery_node.py.swp 
sudo i2cdetect -y 0
sudo i2cdetect -y 1
vim src/sensor_pkg/sensor_pkg/battery_node.py 
ros2 run sensor_pkg battery_pub 
sudo i2cdetect -y 1
ros2 topic pub --once /battery_state/data interfaces/msg/BatteryState "percentage: 42"
sudo i2cdetect -y 1
vim src/sensor_pkg/sensor_pkg/imu_node.py 
colcon build --packages-select sensor_pkg
ros2 run sensor_pkg imu_pub 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
colcon build --packages-select sensor_pkg
ros2 run sensor_pkg imu_pub 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
colcon build --packages-select sensor_pkg
ros2 run sensor_pkg imu_pub 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
colcon build --packages-select sensor_pkg
ros2 run sensor_pkg imu_pub 
wq
vim src/sensor_pkg/sensor_pkg/imu_node.py 
sudo i2cdetect -y 1
ros2 run sensor_pkg imu_pub 
ros2 launch launch_pkg launch.py
vim src/actuator_pkg/actuator_pkg/buzzer_service.py 
ros2 launch launch_pkg launch.py
vim src/vision_pkg/vision_pkg/object_detection_node.py 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg
ros2 run vision_pkg vision_node 
ls src/vision_pkg/vision_pkg/model/o
ls src/vision_pkg/vision_pkg/model/
colcon build --packages-select vision_pkg
ros2 run vision_pkg vision_node 
source install/setup.sh 
ros2 run vision_pkg vision_node 
vim src/vision_pkg/setup.py 
colcon build --packages-select vision_pkg
vim src/vision_pkg/setup.py 
colcon build --packages-select vision_pkg
ls install/vision_pkg/lib/python3.11/site-packages/vision_pkg/model/v5_gray.pt
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg
vim src/vision_pkg/vision_pkg/yolo_detector.py 
vim src/vision_pkg/vision_pkg/object_detection_node.py 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg --symlink-install
vim src/vision_pkg/vision_pkg/yolo_detector.py 
vim src/vision_pkg/setup.py 
colcon build --packages-select vision_pkg --symlink-install
colcon build --packages-select vision_pkg 
vim src/vision_pkg/setup.py 
rm -rf build/ install/ log/
colcon build --packages-select vision_pkg 
vim src/vision_pkg/setup.py 
colcon build --packages-select vision_pkg 
vim src/vision_pkg/setup.py 
colcon build --packages-select vision_pkg 
vim src/vision_pkg/setup.py 
colcon build --packages-select vision_pkg 
vim src/vision_pkg/vision_pkg/model/__init__.py
colcon build
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg 
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg 
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg 
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg 
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg 
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg 
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg 
ros2 run vision_pkg vision_node 
ls
cd ros2_kart_ws/
clear
cd ros2_kart_ws/
vim src/sensor_pkg/sensor_pkg/imu_node.py 
rm src/sensor_pkg/sensor_pkg/.imu_node.py.swp 
colcon build --packages-select sensor_pkg
ros2 run sensor_pkg imu_pub 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
colcon build --packages-select sensor_pkg
ros2 run sensor_pkg imu_pub 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
colcon build --packages-select sensor_pkg
ros2 run sensor_pkg imu_pub 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
vim src/sensor_pkg/setup.py 
vim src/sensor_pkg/package.xml 
colcon build --packages-select sensor_pkg
ros2 run sensor_pkg imu_pub 
vim src/sensor_pkg/package.xml 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
colcon build --packages-select sensor_pkg
ros2 run sensor_pkg imu_pub 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
colcon build --packages-select sensor_pkg
clear
ls
ros2 run decision_pkg decision_node 
cd ros2_kart_ws/
vim src/decision_pkg/decision_pkg/decision_node.py 
vim src/interfaces/msg/Illuminance.msg 
cd ros2_kart_ws/
ls
vim src/actuator_pkg/actuator_pkg/buzzer_service.py 
ros2 launch launch_pkg launch.py
vim src/actuator_pkg/actuator_pkg/buzzer_service.py 
ros2 run actuator_pkg buzzer_srv 
vim src/actuator_pkg/actuator_pkg/buzzer_service.py 
vim src/interfaces/srv/BuzzerCommand.srv 
vim src/actuator_pkg/actuator_pkg/buzzer_service.py 
vim src/actuator_pkg/actuator_pkg/light_service.py 
colcon build
ros2 launch launch_pkg launch.py
vim src/actuator_pkg/actuator_pkg/light_service.py 
ros2 launch launch_pkg launch.py
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg
vim src/actuator_pkg/actuator_pkg/light_service.py 
colcon build --packages-select actuator_pkg
ros2 launch launch_pkg launch.py
vim src/actuator_pkg/actuator_pkg/light_service.py 
colcon build --packages-select actuator_pkg
ros2 launch launch_pkg launch.py
vim src/decision_pkg/decision_pkg/decision_node.py 
vim src/actuator_pkg/actuator_pkg/light_service.py 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
vim src/sensor_pkg/sensor_pkg/illuminance_node.py 
vim src/interfaces/msg/Illuminance.msg
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/interfaces/msg/Illuminance.msg
vim src/sensor_pkg/sensor_pkg/illuminance_node.py 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
vim src/sensor_pkg/sensor_pkg/illuminance_node.py 
vim src/sensor_pkg/sensor_pkg/imu_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/vision_pkg/vision_pkg/object_detection_node.py 
vim src/interfaces/msg/
vim src/vision_pkg/vision_pkg/object_detection_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/sensor_pkg/sensor_pkg/illuminance_node.py 
clear
cd ros2_kart_ws/
vim src/sensor_pkg/sensor_pkg/illuminance_node.py 
rm src/sensor_pkg/sensor_pkg/.illuminance_node.py.swp 
vim src/sensor_pkg/sensor_pkg/illuminance_node.py 
cd ros2_kart_ws/
vim src/interfaces/msg/Illuminance.msg 
rm src/interfaces/msg/.Illuminance.msg.swp 
clear
vim src/decision_pkg/decision_pkg/decision_node.py 
clear
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/decision_pkg/decision_pkg/decision_node.py 
vim src/interfaces/srv/LightCommand.srv 
vim src/interfaces/msg/Config.msg 
vim src/decision_pkg/decision_pkg/decision_node.py 
cd ros2_kart_ws/
vim src/decision_pkg/decision_pkg/decision_node.py 
rm src/decision_pkg/decision_pkg/.decision_node.py.swp 
vim src/decision_pkg/decision_pkg/decision_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/sensor_pkg/sensor_pkg/illuminance_node.py 
clear
cd ros2_kart_ws/
vim src/sensor_pkg/sensor_pkg/illuminance_node.py 
rm src/sensor_pkg/sensor_pkg/.illuminance_node.py.swp 
vim src/actuator_pkg/actuator_pkg/buzzer_service.py 
clear
cd ros2_kart_ws/
vim src/decision_pkg/decision_pkg/decision_node.py 
rm src/decision_pkg/decision_pkg/.decision_node.py.swp 
vim src/decision_pkg/decision_pkg/decision_node.py 
cd ros2_kart_ws/
vim src/vision_pkg/vision_pkg/object_detection_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build
ros2 launch launch_pkg launch.py
vim src/sensor_pkg/sensor_pkg/illuminance_node.py 
ros2 launch launch_pkg launch.py
colcon build
ros2 launch launch_pkg launch.py
colcon build
ros2 launch launch_pkg launch.py
ls
bluetooth
sudo bluetooth
sudo bluetoothctl 
clear
cd ros2_kart_ws/
vim src/decision_pkg/decision_pkg/decision_node.py 
rm src/decision_pkg/decision_pkg/.decision_node.py.swp 
vim src/decision_pkg/decision_pkg/decision_node.py 
colcom build
colcon build
ls
vim ~/config.conf 
ros2 launch launch_pkg launch.py
cd ros2_kart_ws/
clear
vim src/sensor_pkg/setup.py
vim src/sensor_pkg/package.xml 
ls src/interfaces/msg/
ls src/interfaces/srv/
clear
cd ros2_kart_ws/
vim src/actuator_pkg/actuator_pkg/buzzer_service.py 
rm src/actuator_pkg/actuator_pkg/.buzzer_service.py.swp 
vim src/actuator_pkg/actuator_pkg/buzzer_service.py 
vim src/decision_pkg/setup.py 
vim src/decision_pkg/package.xml 
vim src/vision_pkg/package.xml 
vim src/interfaces/msg/
vim src/interfaces/msg/Illuminance.msg 
vim src/interfaces/CMakeLists.txt 
colcon build
cd ros2_kart_ws/
cd ros2_kart_ws/
vim src/vision_pkg/vision_pkg/object_detection_node.py 
cd ros2_kart_ws/
vim src/interfaces/msg/Illuminance.msg 
colcon build
ros2 launch launch_pkg launch.py
cd ros2_kart_ws/
vim src/sensor_pkg/sensor_pkg/illuminance_node.py 
vim src/decision_pkg/decision_pkg/decision_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/decision_pkg/decision_pkg/decision_node.py 
colcon build
ros2 launch launch_pkg launch.py
64;106;42M64;106;42M64;106;42M64;106;42M64;106;38M64;107;36M64;107;36M65;107;36M65;108;36M65;108;36M65;108;36M65;109;36M65;109;36M65;109;36M65;109;36M65;109;36M65;109;36M65;109;36M0;110;37M2;110;37M0;110;37m2;110;37m1;110;37M33;109;37M1;109;37mexit
64;35;34M64;35;34M64;35;34M64;35;34M64;35;34M64;36;34M65;42;34M65;42;34M65;42;34M65;42;34M65;42;34M65;42;34
exit
vim ./.bashrc 
cd ros2_kart_ws/
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
vim ~/ros2_humble_venv/lib/python3.11/site-packages/bluezero/peripheral.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
colcon build --packages-select app_interface_pkg
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_node.py 
colcon build --packages-select app_interface_pkg
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
vim src/app_interface_pkg/app_interface_pkg/ble_periph.py 
colcon build --packages-select app_interface_pkg
ros2 run app_interface_pkg ble_node 
vim src/app_interface_pkg/app_interface_pkg/ble_agent.py 
ls
cd ros2_kart_ws/
vim src/decision_pkg/decision_pkg/decision_node.py 
poweroff 
sudo poweroff 
i2cdetect -y 1
sudo poweroff 
cd ros2_kart_ws/
vim src/sensor_pkg/sensor_pkg/imu_node.py 
sudo poweroff 
ros2 launch launch_pkg launch.py
ros2 run vision_pkg vision_node 
vim ros2_kart_ws/src/vision_pkg/vision_pkg/object_detection_node.py 
vim ros2_kart_ws/src/interfaces/msg/
vim ros2_kart_ws/src/interfaces/msg/Vision.msg 
cd ros2_kart_ws/
colcon build
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/model/
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg
ros2 run vision_pkg vision_node 
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg
vim src/vision_pkg/vision_pkg/yolo_detector.py 
colcon build --packages-select vision_pkg
ros2 run vision_pkg vision_node 
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  vim start_ros2_launch.sh 
vim /etc/systemd/system/ros2-auto-launch.service 
vim start_ros2_launch.sh 
vim .bashrc 
vim start_ros2_launch.sh 
vim /etc/systemd/system/ros2-auto-launch.service 
sudo vim /etc/systemd/system/ros2-auto-launch.service 
sudo systemctl restart ros2-auto-launch.service 
sudo systemctl daemon-reload
sudo systemctl enable ros2-auto-launch.service 
sudo systemctl restart ros2-auto-launch.service 
ros2 launch launch_pkg launch.py
cd ros2_kart_ws/
vim src/vision_pkg/vision_pkg/object_detection_node.py 
sudo vim /usr/lib/python3/dist-packages/picamera2/picamera2.py 
libcamera-hello
sudo systemctl status ros2-auto-launch.service 
ps aux | grep ros2
sudo systemctl stop ros2-auto-launch.service 
vim ros2_kart_ws/src/actuator_pkg/actuator_pkg/light_service.py 
vim ros2_kart_ws/src/actuator_pkg/actuator_pkg/buzzer_service.py 
sudo systemctl status ros2-auto-launch.service 
poweroff 
ls
ls -al
git add .
git rm --cached ros2_kart_ws/src/vision_pkg/vision_pkg/yolo5
vim .gitignore 
ls ros2_kart_ws/src/vision_pkg/vision_pkg/yolo5
cd ros2_kart_ws/src/vision_pkg/vision_pkg/yolov5/
ls
cd ~
ls ros2_kart_ws/src/vision_pkg/vision_pkg/yolo5/
ls ros2_kart_ws/src/vision_pkg/vision_pkg/yolov5
vim .gitignore 
git add .
git commit -m "semi_final"
git push
ls /etc/services 
cd /etc/services
ls .
ls /
ls /srv/
ls /etc/services 
ls
vim start_ros2_launch.sh 
ls /etc/systemd/system
vim /etc/systemd/system/ros2-auto-launch.service 
sudo systemctl disable ros2-auto-launch.service 
