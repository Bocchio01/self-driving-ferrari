# RaspberryPi Ubuntu 26.04 Configuration

This is the setup guide for a RaspberryPi running Ubuntu 26.04 for the first time.

## MicroSD card

Get the RaspberryPi Imager from https://www.raspberrypi.com/software/ and install it on your computer.

```bash
sudo apt install rpi-imager
```

Select Raspberry Pi 4 Model B 64-bit and Ubuntu Server 26.04 LTS (64-bit).
As for the settings, set the hostname to `raspberypi`, set the username to `bocchio`, and set the password to your desired password.
Set WiFi SSID and password, and enable SSH.

Run the imager and write the image to the microSD card.
Once the image is written, insert the microSD card into the RaspberryPi and power it on.

At least in my case, the RaspberryPi will not connect to WiFi on the first boot as it seems to enter a "first boot" mode.
Leave it running for a few minutes, then power it off and on again. It should connect to WiFi on the second boot.


## Configure the RaspberryPi

At first, connect to the RaspberryPi via SSH.
You can find the IP address of the RaspberryPi by checking your router's connected devices list.

```bash
ssh bocchio@<IP_ADDRESS>
```

On startup, Ubuntu will try to download all the security updates and patches.
`sudo apt upgrade` command can't be executed until those are done.
To check status:

```bash
sudo tail -f /var/log/unattended-upgrades/unattended-upgrades-dpkg.log
```

Once the updates are done, you can run the following commands to update and upgrade the system:

```bash
sudo apt update
sudo apt upgrade -y
```

Also install some useful packages:

```bash
sudo apt install tree
curl https://getmic.ro | bash # This will install micro editor in the current directory
sudo mv micro /usr/bin # Move it to /usr/bin so you can use it from anywhere
```


### Change the host name

If you want to change the hostname of your RaspberryPi, you can do so with the following command:

```bash
sudo hostnamectl set-hostname raspberypi
```

Then we also need to inform the net about this change by editing the `/etc/hosts` file:

```bash
sudo micro /etc/hosts
```

Right below the line `localhost`, add the line `127.0.1.1	raspberypi`, so that the file looks like this:

```bash
127.0.0.1	localhost
127.0.1.1	raspberypi
::1		localhost ip6-localhost ip6-loopback
ff02::1		ip6-allnodes
ff02::2		ip6-allrouters
```

By default, Ubuntu 26.04 doesn't come with `avahi-daemon` installed, which is needed for the hostname to be resolved on the network.
Install it with the following command:

```bash
sudo apt install avahi-daemon
```

Check the status of the service:

```bash
sudo systemctl status avahi-daemon
```


### Setup SSH connection keys

To avoid having to enter your password every time you connect to the RaspberryPi via SSH, you can set up SSH keys.

On your local machine, generate a new SSH key pair if you don't have one already:

```bash
ssh-keygen -t ed25519 -f ~/.ssh/id_raspi
```

Then copy the public key to the RaspberryPi:

```bash
ssh-copy-id -i ~/.ssh/id_raspi.pub bocchio@<IP_ADDRESS>
```

It's also useful to have a shortcut in your SSH config file for easier access.
Edit (or create) the `~/.ssh/config` file on your local machine and add the following:

```
Host raspberrypi
    HostName raspberrypi.local # Or use the IP address of your RaspberryPi
    User bocchio
    IdentityFile ~/.ssh/id_raspi
    ForwardX11 yes
    ForwardX11Trusted yes
```


### Setup SSH github keys

Login to your RaspberryPi and generate a new SSH key pair for GitHub:

```bash
ssh-keygen -t ed25519 -C "tommaso.bocchietti@gmail.com"
```

Then copy the public key to your clipboard:

```bash
cat ~/.ssh/id_ed25519.pub
```

Go to your GitHub account settings, navigate to "SSH and GPG keys", and add a new SSH key with the contents of your public key.
Basically, just copy the output of the previous command and paste it into GitHub.
Call the key `RaspberryPi4B` or something similar.


### ROS installation

Before installing ROS, it's best to reboot the RaspberryPi to ensure all updates are applied:

```bash
sudo reboot
```

Then, following the guide from the official (ROS website)[https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html], install ROS 2 Lyrical Luth on on your RaspberryPi.

```bash
locale # Check locale if not UTF-8, run the following commands
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locales | grep UTF-8 # Verify settings
```

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

```bash
sudo apt update && sudo apt install ros-dev-tools # This will take a while...
```

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-lyrical-ros-base # This will take a while...
```

Finally, we add the ROS setup script to the `.bashrc` file so that it is sourced every time we open a new terminal:

```bash
echo "source /opt/ros/lyrical/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Additional ROS packages

For `colcon` autocomplete, we need to install the following package:

```bash
sudo apt install python3-colcon-argcomplete
```

And source the `colcon` setup script in the `.bashrc` file:

```bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```


## Ferrari project setup

Now, we can set up the Ferrari project on the RaspberryPi.

We will clone the repository in the home directory of the `bocchio` user:

```bash
cd ~
git clone git@github.com:Bocchio01/self-driving-ferrari.git
cd self-driving-ferrari/software
```

Then we will install the dependencies using `rosdep`:

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y # This will take a while...
```

Then, we will build the project using `colcon`:

```bash
colcon build --symlink-install
```

### Docker installation

For the `ferrari_vehicle` package, we need to install Docker on the RaspberryPi, at least for now, as it is the easiest way to run the `microROS` agent for the ROS2 `jazzy` distro.

```bash
sudo apt install docker.io
sudo systemctl enable docker
sudo systemctl start docker
sudo usermod -aG docker $USER
```

Notice that you will need to log out and log back in for the group change to take effect.


### Serial communication over GPIO (bluetooth disabled)

To enable serial communication over GPIO, we need to disable the Bluetooth module on the RaspberryPi.
To do so, we follow the guide provided here: (osr-rover-code)[https://github.com/nasa-jpl/osr-rover-code/blob/foxy-devel/setup/rpi.md]

Disable serial-getty@ttyS0.service

```bash
sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl disable serial-getty@ttyS0.service
sudo systemctl mask serial-getty@ttyS0.service
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@ttyAMA0.service
sudo systemctl mask serial-getty@ttyAMA0.service
```

Write the following udev rules to `/etc/udev/rules.d/10-local.rules`:

```bash
#   See https://www.engineersgarage.com/microcontroller-projects/articles-raspberry-pi-serial-communication-uart-protocol-serial-linux-devices/
KERNEL=="ttyAMA0", SYMLINK+="serial0" GROUP="tty" MODE="0660"
KERNEL=="ttyS0", SYMLINK+="serial1" GROUP="tty" MODE="0660"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Add user to tty and dialout groups

```bash
sudo adduser $USER tty
sudo adduser $USER dialout # Will probably give a warning, but it's ok
```

Remove console line in cmdline.txt boot config file.
This is actually already done in the Ubuntu 26.04 image, but just in case, check that the line `console=serial0,115200` is not present in `/boot/firmware/cmdline.txt`.

Disable bluetooth in config.txt boot config file by adding `dtoverlay=disable-bt` to `/boot/firmware/config.txt`.
Place it at the very end of the file, after all other settings, so that it is not overridden by any other setting.

```bash
# Other settings in config.txt

[all]
dtoverlay=disable-bt
enable_uart=1
```

Finally, reboot the RaspberryPi to apply the changes:

```bash
sudo reboot
```


### Camera setup (IMX219)

To make the camera work, we need to install the following packages:

```bash
sudo apt update
sudo apt install \
    gstreamer1.0-tools \
    gstreamer1.0-libcamera \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good
```

Moreover, we need to modify the following line to `/boot/firmware/config.txt`:

```bash
camera_auto_detect=0
```

And add the following line at the end of the file to enable the IMX219 camera:

```bash
#Other settings in config.txt

[all]
dtoverlay=imx219

#Other settings in config.txt
```



