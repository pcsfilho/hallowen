# Hallowen V-REP
This repository presents a project which establishes a connection between the client side developed with the help of python language and the VREP tool. It implemented the logic in a quadcopter should find a black cube on the map.

## Tools and Requirements

**Vrep**:
[V-REP](http://www.coppeliarobotics.com/) is a robot simulator, with integrated development environment.Controllers can be written in C/C++, Python, Java, Lua, Matlab or Octave. The version used in this project was V-REP 3.6.2.

**Python**:
Python version v3.6 or greater 

**Operational system**
Linux 64x

## Run
- **First first you must download the project in git:**

  ```bash
    $ git clone https://github.com/pcsfilho/hallowen.git
    ```
- **After downloading the project you must enter the folder:**

    ```bash
        $ cd hallowen
    ```

- **You must open the scene in VREP. The scene is located in the vrep_content folder and is named desafio.ttt.**
- **After opening the scene and starting the scene, you can start the python script for the control and execution of the map search.**:

    ```bash
        $ python main
    ```

## Project
This project is divided into two main classes:
- **Map**
    - Represents the Map with its respective attributes and methods.
        - Atributes:
            - clientID : Parameter id of the client with api remote
            - x: Parameter array containing map boundaries on x axis
            - y: Parameter array containing map boundaries on y axis
            - z: Parameter array containing map boundaries on z axis
        - Methods:
            - start_connection(server, port): Method that initiates the connection with server
            - stop_connection(server): Method that stop the connection with server
        


- **Quadcopter**
Represents the quadcopter with attributes and sensors.
    - Atributes:
        - target: Target that control the quadcopter position.
        - sensor_vision:
        - shift:
        - proximity_sensor_1: This proximity sensor is responsible for detecting collisions in forward movements
        - proximity_sensor_2: This proximity sensor is responsible for detecting collisions in backard movements
        - proximity_sensor_3: This proximity sensor is responsible for detecting collisions in right move
        - proximity_sensor_4: This proximity sensor is responsible for detecting collisions in left move
        - proximity_sensor_5: This proximity sensor is responsible for detecting collisions in downward movements
        - vision_sensor: This sensor is responsible for detecting the cube with reference color 23 in the map
    - Methods:
        - object_was_found: verify if exist reference color in the image generated by sensor vision
        - test_trajectory test if any proximity sensor is true and corrects trajectory related to previous movement
        - initial_position: put quadcopter on initial position (height and minimun x) before the search.
        - make_trajectory: causes the quadcopter to rotate the map until it finds the cube

## Documentation
This project was based on
- [VREP](http://www.coppeliarobotics.com/helpFiles/)
- [Forum](http://www.forum.coppeliarobotics.com)

## Project decisions
    - For this project defined the following settings for the vision sensor quadcopter:

![Optional Text](https://github.com/pcsfilho/hallowen/blob/master/img/pespective.png)

    - Just like a color selection filter for cube visualization in simulation:

![Optional Text](https://github.com/pcsfilho/hallowen/blob/master/img/filter.png)

## Improvements
In the future it is expected to improve the search algorithm in order to find the object more accurately.
In addition to improving collision correction.