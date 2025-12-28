### PERCEPTION IN YOUBOT


For here, the idea is to integrate the YOLOv8 model to detect object, navigate to it, pick it up, and put it somewhere else or give it to a human.

+--------------------+
|      Perception    |
|  YOLO + Camera     |
|  Object detection  |
+----------+---------+
           |
           v
+--------------------+
| Object Localization|
| 2D → 3D pose       |
+----------+---------+
           |
           v
+--------------------+
| Navigation (Nav2)  |
| Base positioning   |
+----------+---------+
           |
           v
+----------------------+
| Manipulation         |
| MoveIt / IK  / DRL   |
+----------+-----------+
           |
           v
+--------------------+
| Task Logic         |
| FSM / BT           |
+--------------------+


First I will try to integrate the YOLO model 

Due to the small size of the gripper, I am going to train the model on nails

Then, after successfull integration with ros2, I am going to add the navigation, so the robot should navigate to it, finally should pick it up. 


