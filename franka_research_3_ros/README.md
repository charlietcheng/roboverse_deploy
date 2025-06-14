
#### Charlie's franka deploy tool based on Ros1. Currently only fr3 and franka panda emika is supported

## BUILD DOCKER

```
cd <Project Dir>
git submodule update --init 

cd docker
docker compose build franka_controller
```

## CONNECT TO FRANKA

### Finding IP addr of franka arm and the control box
tested on the latest franka arm (2025-03-16) with libfranka version 0.15.0: connect the ethernet cable to the franka arm directly to fetch the control box ip address (network page -> Shop Floor network -> Current network status)

1: connect the ethernet cable to the franka arm directly. Set the dchp network ip to a custom ip addr (not that 172.xx.xx.xx might not work if directly connected to the franka arm/control box)

2: directly connect the ethernet cable to the control box. Connect the control box with the ip found earlier. 

### If ip still not found: use a router/switch to troubleshoot
## OPEN FRANKA FCI
open with web browser : `https://172.16.0.1/desk/` (or custom location)

then unlock franka

## RESET FRANKA
```
# reset franka
docker compose run reset
```

## openvla experiment
Policy server default port: 26784


## DEBUG 

connect to container

```
docker run --name franka -it  franka_control:latest
```

### default franka position ik result:
```
(array([ 1.09388986e-01, -6.98604344e-17,  5.84831358e-01]), array([[ 6.93011723e-01,  6.93011723e-01, -1.98669331e-01],
       [ 7.07106781e-01, -7.07106781e-01, -9.88337884e-18],
       [-1.40480431e-01, -1.40480431e-01, -9.80066578e-01]]))
```

#### Qt platform problem:

If you see: 
```
[+] Creating 2/2
 ✔ Container franka-control-private-roscore-1            Created                                                          0.0s 
 ✔ Container franka-control-private-franka_controller-1  Create...                                                        0.0s 
[+] Running 2/2
 ✔ Container franka-control-private-roscore-1            Started                                                          0.1s 
 ✔ Container franka-control-private-franka_controller-1  Starte...                                                        0.1s 
qt.qpa.xcb: could not connect to display 
qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "" even though it was found.
This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem.

Available platform plugins are: eglfs, linuxfb, minimal, minimalegl, offscreen, vnc, xcb.

exit status 139
```

run:
```
xhost +local:docker
```


```
docker compose run roscore
docker compose run franka_controller


ssh -L 26784:0.0.0.0:26784 ghr@em10.ist.berkeley.edu

docker compose run main main --agent=openvla

--> head shoulders supreme


docker compose run reset
```
