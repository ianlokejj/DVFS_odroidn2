# Embedded Software Design Project
An embedded software design project for my masters.

## Introduction
The goal of this project is to build an FPS-Aware Governor that runs on userspace. The objective of
the Governor is to ensure that the target FPS and achieved FPS are as close as possible. If the
achieved FPS is lower than the target FPS, the Governor is considered to have failed to meet the QoS requirement. On the other hand, if the achieved FPS is higher than the target FPS, the Governor is wasting power by over-provisioning resources.

![Architecture](/images/architecture.png)

The governor makes use of DVFS (Dynamic Voltage and Frequency Scaling) and CPU affinity as knobs to achieve the target FPS.

The project was developed on the embedded hardware platform ODROID-N2 which is capable of heterogeneous processing. It consists of 4 ARM A73 cores, 2 ARM A53 cores and one ARM Mali-G52 GPU.

![ODROID-N2](/images/odroid-N2.jpg )

## Governor design

The governor is implemented using Finite State Machines (FSM) solution with online profiling technique.

The Sampler FSM acts as a front-end processor to filter the incoming measured FPS (provided by
the main application) to smoothen the short-time variation in system performance.

The Profiler FSM performs online profiling by maintaining a hash table which maps the combinations of system resources (i.e. frequency settings, number of cores, cluster used) to their achieved FPS values.

The Main FSM adjusts the system configuration (i.e. frequency settings, number of cores, cluster used)

![Combined FSM](/images/combined_fsm.png)

## Overall Design

![Design](/images/cluster_migration.png)

## Set up the project
1. Download the Odroid N2 Ubuntu MATE image from https://dn.odroid.com/S922X/ODROID-N2/Ubuntu/. The project was developed and tested on `ubuntu-18.04.4-4.9-mate-odroid-n2-20200224.img.xz`
2. Extract and write the image onto an SD Card. You can use Ubuntu Startup Disk Creator.
3. Connect the board to your local network and `ssh` into the N2 with the `-XY` command i.e `ssh -XY odroid@IP_ADDRESS`. The default password is `odroid`.
4. Update and upgrade the board by running `sudo apt update && sudo apt upgrade`.
5. Install [OpenCV](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html).  on the board. *This project was developed on OpenCV 4.3.0. You do not need to install the optional packages.*
6. Clone this repository onto the board. `git clone https://github.com/ianloke/DVFS_odroidn2.git`
