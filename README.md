---
title: 'Keyboard Control drone(Python)'
disqus: hackmd
---

Keyboard Control drone(python script)
===


[TOC]

## 1. Why
需測試關於Mavlink指令所以需要有一份Python Script去控制Drone，Python lib使用DroneKit可以提供一些連線以及切換模式的簡單指令，但是Drone的移動還是需要用Mavlink撰寫，測試可使用虛擬環境(SITL)是Pixhawk實際測試

:::warning
SITL建置請參考這篇
:::

## 2. DroneKit
### 1.連線
連線則可以是使用TCP或是COM去做連結，如果是使用DroneKit SIT模擬，則可以使用TCP連結到虛擬的Drone。如果是使用真實的Drone則需要看數傳連接到電腦的哪一個COM，COM則可以在裝置管理員中的連接埠(COM和LPT)確認當下數傳是對應到哪一個COM
```
vehicle = connect('tcp:127.0.0.1:14551')
vehicle = connect('com',baud=51200)
```
### 2.切換模式(Guided, Land)
在DroneKit當中可以使用VehicleMode去做切換，所以在使用Mavlink腳本之前，基於我們是地面站控制，所以需要先切換到GUIDED MODE去做控制，那在降落的時候也可以切換到LAND MODE方便降落
```
vehicle.mode = VehicleMode("GUIDED")
vehicle.mode = VehicleMode("LAND")

```
### 3.解鎖(Arm)
使用DroneKit當中的vehicle.armed去做解鎖，再進行接來下的控制
```
 vehicle.armed = True
```

## 3. Mavlink
### 1.方向性(Roll, Pitch, Throttle)
我們希望可以使用鍵盤的WASD↑↓對應Drone的操作，這邊所選用Mavlink中的速度的指令，mavlink.MAV_FRAME_BODY_NED會改變Drone對於X, Y, Z軸的速度，所以我們可以針對速度去做一個改變，例如:按下"W"按鍵給予X軸vx->0.2m/s的速度改變，按下"A"按鍵給予Y軸vy->0.2m/s改變，按下"↑"按鍵給予Z軸vz->0.2m/s的速度改變，這樣可以達到按下相對應按鍵移動相對應方向
```
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:
    
    
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
```
```
set_velocity_body(vehicle, gnd_speed, 0, 0)
set_velocity_body(vehicle, 0, gnd_speed, 0)
set_velocity_body(vehicle, 0, 0, gnd_speed)
```
### 2.機頭方向(Yaw)

我們希望可以使用鍵盤的QE對應Drone的操作，這邊所選用Mavlink中的Yaw的指令，
mavlink.MAV_CMD_CONDITION_YAW會改變Drone對於YAW的角度，所以我們可以針對角度去做一個改變，例如:按下"Q"按鍵給予Yaw去做一個負10度的改變，按下"E"按鍵給予Yaw去做一個正10度的改變，這樣可以達到按下相對應按鍵移動相對應機頭方向
```
def condition_yaw(heading, relative, direction):
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        direction,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
```
            
```           
condition_yaw(10, True, -1)#對應Yaw負10度轉動
condition_yaw(10, True, 1)#對應Yaw正10度轉動
```
## 3. Relay

args[1]代表<RELAY_NUM> args[2]代表<0|1>

```
mavutil.mavlink.MAV_CMD_DO_SET_RELAY, 0,
int(args[1]), int(args[2]),
0, 0, 0, 0, 0)

```
參考資料 
---


1. https://mavlink.io/en/messages/common.html
2. https://dronekit-python.readthedocs.io/en/latest/examples/guided-set-speed-yaw-demo.html
3. https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
###### tags: `setup`,




