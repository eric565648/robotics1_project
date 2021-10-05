# Kinova Arm Setup
Last edited: Oct. 5 2021

## Hardware Setup

1. Plugin the powercable and the joystick
2. Connect the USB cable to your laptop
3. Powerup the arm

The green light on the joystick will be flashing for around a minute. After that, there will be two results after you do that
### Joystick Steady Green light
You are good to go! Please direct to [software setup](#software-setup).

### Joystick Green Light flashing or Steady Red Light
You need to update the firmware of the arm. If you try to use the ROS package when this situation happens, it'll show connection error.

1. Please follow the instruction in this [webpage](https://github.com/Kinovarobotics/kinova-ros/issues/248).
2. You can find the latest firmware [here](https://www.kinovarobotics.com/en/resources/gen2-technical-resources).
3. You can find the installation files of Development Center, a GUI available with the SDK [here](https://www.kinovarobotics.com/en/resources/gen2-faq) in the first question of "Usage". If the download link is missing, you can find it [here](https://drive.google.com/file/d/1UEQAow0XLcVcPCeQfHK9ERBihOCclkJ9/view) and the document [here](https://www.kinovarobotics.com/sites/default/files/UG-008_KINOVA_Software_development_kit-User_guide_EN_R02%20%281%29.pdf).
4. We only tested the Development Center in win10 while updating the firmware. However, Development Center in both win10 and ubuntu20.04 (using the ubuntu16.04 version in the file) can connected to the arm and control the arm.
5. While rebooting the arm in the updating firmware step, you might want to do some "physical therapy" for the arm (basically move the arms around) and wait a bit before restarting your arm.

## Software Setup

### Control the arm via Development Center
You can find the installation files of Development Center, a GUI available with the SDK [here](https://www.kinovarobotics.com/en/resources/gen2-faq) in the first question of "Usage". If the download link is missing, you can find it [here](https://drive.google.com/file/d/1UEQAow0XLcVcPCeQfHK9ERBihOCclkJ9/view) and the document [here](https://www.kinovarobotics.com/sites/default/files/UG-008_KINOVA_Software_development_kit-User_guide_EN_R02%20%281%29.pdf).

### Control the arm via ROS package
Please direct [here](https://github.com/eric565648/kinova-ros/tree/noetic-devel) for how to launch the arm, perform joint angle and cartetian space control.

#### Note
1. The admittance control does not work.
2. We have not do the torque calibration yet.