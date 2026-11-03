# Raspberry Pi 5 setup

The arm, the camera and the web dashboard run on a Pi 5 with no screen; you
use it from a browser. Use **Ubuntu Server 24.04 (64-bit)**: Raspberry Pi OS
has no ROS 2 Jazzy packages. Every command below runs on the Pi, in an SSH
session, in order. Allow about 30 minutes (most of it downloads).

**Hardware:** Pi 5 with the official 5 V / 5 A USB-C supply (a weaker one
limits USB current), a 32 GB+ microSD card, the Arduino UNO + Braccio shield
with its own 5 V / 4 A+ servo supply, and the overhead camera (a Grove Vision
AI V2 with its Raspberry Pi camera, or a USB webcam). Plug the UNO and the
camera straight into the Pi, or use a **powered** USB hub: an unpowered hub
can drop both boards mid-run.

## 1. Flash the SD card (on your PC)

In [Raspberry Pi Imager](https://www.raspberrypi.com/software/): *Device*
Raspberry Pi 5 → *OS* Other general-purpose OS → Ubuntu → **Ubuntu Server
24.04 LTS (64-bit)** → *Storage* your card. When it asks to apply OS
customisation, choose **Edit settings** and set:

- hostname: `braccio` (the Pi is then `braccio.local` on the network)
- a username and password
- your Wi-Fi name, password and country (or use an Ethernet cable)
- *Services*: **Enable SSH** with password authentication

Write the card, put it in the Pi, power it on and wait about 2 minutes.

## 2. Log in and update

From your PC (Linux, macOS or Windows PowerShell):

```bash
ssh <username>@braccio.local          # or ssh <username>@<the Pi's IP address>
```

On the Pi:

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y git curl tmux avahi-daemon locales software-properties-common
sudo reboot
```

Log in again after about a minute (`ssh <username>@braccio.local`).

## 3. Install ROS 2 Jazzy (the light version)

These follow the [official ROS 2 Jazzy install](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html),
but install `ros-base` (no desktop, RViz or Gazebo):

```bash
# a UTF-8 locale, as ROS 2 asks
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# the ROS 2 apt source
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# ROS 2 base and everything this project uses on the Pi
sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-jazzy-xacro ros-jazzy-cv-bridge \
  python3-colcon-common-extensions python3-opencv python3-serial python3-yaml \
  python3-numpy python3-pip

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
ros2 pkg list | head -3               # check: prints package names
```

If the ROS apt source step fails, use the commands on the official page
linked above (they change from time to time); the rest stays the same.

## 4. Get the code and build it

```bash
cd ~
git clone https://github.com/hwivc/unoq-braccio.git
git clone https://github.com/hwivc/ROS--2-Ten-Months-Challenge.git      # Grove Vision AI V2 driver

# the Grove driver (its dependencies are already installed from apt above)
pip install --break-system-packages --no-deps -e ~/ROS--2-Ten-Months-Challenge/test_camera

# build only the three packages the arm needs (rosdep is skipped on purpose: it would install Gazebo)
cd ~/unoq-braccio/ros2_ws
colcon build --symlink-install --packages-select unoq_braccio_sim unoq_braccio_driver unoq_braccio_bringup

echo "source ~/unoq-braccio/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

With `--symlink-install`, a later `git pull` needs no rebuild for Python or
web page changes (rebuild after new files or new programs appear).

## 5. USB names and permission

```bash
bash ~/unoq-braccio/scripts/setup_uno_serial.sh
sudo reboot                            # so the new 'dialout' group applies
```

This gives the UNO `/dev/braccio` and the Grove board `/dev/grove_vision`,
lets you open them without `sudo`, and stops ModemManager probing them.

## 6. Flash the UNO (once)

Plug the UNO into the Pi (servo power can stay off), then:

```bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$HOME/.local/bin sh
echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc && source ~/.bashrc

bash ~/unoq-braccio/scripts/flash_uno.sh /dev/braccio
```

The first time it downloads the Arduino AVR tools (a few minutes). For an
UNO R4 Minima add the board name:
`bash ~/unoq-braccio/scripts/flash_uno.sh /dev/braccio arduino:renesas_uno:minima`.
You can flash from a PC instead (see [Real arm](../README.md#real-arm)).

## 7. Check the boards

Plug in the UNO (servo power on) and the camera:

```bash
lsusb | grep -E "2341|1a86"            # 2341:0043 = UNO, 1a86:55d3 = Grove Vision AI V2
ls -l /dev/braccio /dev/grove_vision   # both should exist
ls /dev/video*                         # a USB webcam, if you use one
```

## 8. Start it

```bash
ros2 launch unoq_braccio_bringup pi.launch.py
```

This is [the one command](../README.md#quick-start): arm, camera, cube detection
and the web dashboard. After about 10 s the arm stands up and the terminal
prints the address. Open **`http://braccio.local:8000`** (or the printed
address) on a phone or laptop on the same network and enter the **pairing
code** the terminal shows: that creates the owner account. For a USB webcam:
`pi.launch.py camera:=0`. Stop it with Ctrl+C.

Then, in the browser: Sort → **Add colour** (tap each cube colour once),
Train → teach a few examples, Collect → **Sweep all** for the rest, Show →
start the exhibition. See [Web dashboard](web-dashboard.md).

## 9. Start it at boot (recommended)

So it starts when the Pi is powered on and restarts if it stops, with no SSH
needed:

```bash
sudo tee /etc/systemd/system/braccio.service > /dev/null <<UNIT
[Unit]
Description=Braccio arm and web dashboard
After=network-online.target
Wants=network-online.target

[Service]
User=$USER
ExecStart=/bin/bash -lc 'source /opt/ros/jazzy/setup.bash && source $HOME/unoq-braccio/ros2_ws/install/setup.bash && exec ros2 launch unoq_braccio_bringup pi.launch.py'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
UNIT
sudo systemctl daemon-reload
sudo systemctl enable --now braccio
```

Day to day:

```bash
systemctl status braccio               # is it running?
journalctl -u braccio -f               # its log (the web page's live log shows the same)
sudo systemctl restart braccio         # after an update or a settings change
sudo systemctl stop braccio            # before running it by hand (step 8) or the terminal tools
sudo systemctl disable braccio         # stop starting it at boot
```

The pairing code is in the log the first time: `journalctl -u braccio | grep -i pairing`.

## 10. Update later

```bash
cd ~/unoq-braccio && git pull
cd ros2_ws && colcon build --symlink-install --packages-select unoq_braccio_sim unoq_braccio_driver unoq_braccio_bringup
sudo systemctl restart braccio         # or Ctrl+C and start it again
```

Reload the web page afterwards. Sessions, colours, accounts and settings live
in `~/.ros/` and are kept across updates (back that folder up to keep your
teaching).
