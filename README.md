
# !!ROS2 KURULUMU YAPILMIŞ OLMALIDIR!!

## TEKNOFEST SIMULATION PACKAGE

### GAZEBO CLASSIC KURULUMU

Gazebo'yu aşağıdaki tek satırla kurabilirsiniz. Kuruluysa atlayabilirsiniz.

```bash
curl -sSL http://get.gazebosim.org | sh
```

Kurulum sonrasında tamamlandığını kontrol etmek için aşağıdaki kodu yazdığınızda gazebo'nun açılması gerekir.

```bash
gazebo
```

### SIMULATION VE ROBOTAKSI_AUTONOMOUS_CAR PAKETLERİ

İlk olarak ros2_ws oluşturmadıysanız oluşturmanız gerekiyor. Halihazırda varsa atlayabilirsiniz.

```bash
cd ~
mkdir -p ros2_ws/src
```

Sonrasında gerekli paketleri git kullanarak çekmeniz gerekiyor.

```bash
cd ~/ros2_ws/src
git clone https://github.com/KTU-OTONOM/simulation.git
git clone https://github.com/KTU-OTONOM/via_description.git
```

src içerisindeki paketlerin bağımlılıklarını kurmamız gerekiyor
```bash
cd ~/ros2_ws
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src/simulation src/via_description -y --ignore-src
```

sonrasında ros2_ws içerisine girip kurabilirsiniz

```bash
sudo apt-get install python3-colcon-common-extensions
colcon build --symlink-install --packages-select simulation via_description
echo "source $HOME/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

simülasyonu 3 komutun biriyle çalıştırabilirsiniz
```bash
ros2 launch simulation via_launch.py
ros2 launch simulation ilkel_harita_launch.py
ros2 launch simulation map.launch.py
ros2 launch simulation new_map.launch.py
```
