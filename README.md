# sf_head
Required sf_controller, dynamixel_motor

To lunch sf_head:
    roslaunch sf_head sf_head.launch

To test sf_head:
    rostopic pub -1 /sf_head/sf_headMovement_filename std_msgs/String "Happy.xml" 

