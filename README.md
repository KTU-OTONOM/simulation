
# !!ROS2 KURULUMU YAPILMIŞ OLMALIDIR!!

## TEKNOFEST SIMULATION PACKAGE

### GAZEBO CLASSIC KURULUMU

Gazebo'yu aşağıdaki tek satırla kurabilirsiniz.

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
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

sonrasında ros2_ws içerisine girip kurabilirsiniz

```bash
colcon build --symlink-install --packages-select simulation robotaksi_classic
```

simülasyonu çalıştırabilirsiniz
```bash
ros2 launch simulation tfest.launch.py
```
