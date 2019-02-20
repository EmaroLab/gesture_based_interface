# Gesture-based interface for Baxter Robot

## Authors
| Name | E-mail |
|-----------------------|------------------|
| Antonino Bongiovanni | antoniobongio@gmail.com |
| Alessio De Luca | alessio.deluca.iic96d@gmail.com |
* Luna Gava: lunagava@me.com
* Alessandro Grattarola: alessandro.grt@gmail.com
* Lucrezia Grassi: lucre.grassi@gmail.com
* Marta Lagomarsino: marta.lago@hotmail.it
* Marco Lapolla: marco.lapolla5@gmail.com
* Antonio Marino: marinoantonio96@gmail.com
* Patrick Roncagliolo: roncapat@gmail.com
* Federico Tomat: tomatfede@gmail.com
* Giulia Zaino: giuliazaino46@gmail.com

## Download instructions
```
bash <(wget -qO- https://gist.githubusercontent.com/roncapat/92b6d76c29e5ad35e0647bc6c8c5630f/raw/3e4438901b470a8968e598fdc028d0c4a489da5a/web_installer.sh)
```

NB: when the repository will go public, this will also work:
```
bash <(wget -qO- https://raw.githubusercontent.com/EmaroLab/gesture_based_interface/master/prerequisites.sh)
```

## How to run the simulator
Enter in the workspace
```
cd ~/sofar_ws
```

Execute the file passing as parameter "sim". In this way you can use the simulator in your machine.
If instead you want to work on the real robot you have to pass the serial number of the Baxter.
```
./baxter.sh sim
```

Launch all the nodes used for the simulation.
```
roslaunch baxter_gazebo baxter_world.launch
```

Launch the nodes used for managing the movements and the files.
```
rosrun BaxterGBI_pbr pbr_server_baxter.py
rosrun BaxterGBI_pbr pbr_server_filesys.py
rosrun BaxterGBI_pbr joint_recorder_node.py
rosrun BaxterGBI_pbr mirror_server.py
```


Then you can test it using the following client nodes:
```
rosrun BaxterGBI_pbr pbr_client_TEST.py mode arg1 arg2 ...

rosrun BaxterGBI_pbr mirror_client.py
```

Where, based on mode you can ask for a specific server (and you have to pass specific parameters).


## Kinect launcher
```
roscore
```
Launch nodes for the Kinect
```
roslaunch openni_launch openni.launch device_id:=&lt;device id&gt;
```
```
Rviz, if the Baxter is not shown, run export LC_NUMERIC='en_US.UTF-8' on the shell
```
