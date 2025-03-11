
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
git clone https://github.com/Albaryan/robotaksi_autonomous_car.git
git clone https://github.com/KTU-OTONOM/simulation.git
```

src içerisindeki paketlerin bağımlılıklarını kurmamız gerekiyor
```bash
cd ~/ros2_ws
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src/robotaksi_autonomous_car/robotaksi_autonomous_car src/simulation -y --ignore-src
```

sonrasında ros2_ws içerisine girip kurabilirsiniz

```bash
sudo apt-get install python3-colcon-common-extensions
colcon build --symlink-install --packages-select simulation robotaksi_autonomous_car
echo "source $HOME/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

simülasyonu çalıştırabilirsiniz
```bash
ros2 launch simulation tfest.launch.py
```
