1. install openrave http://ros.org/wiki/openrave
2. generated the dae file from the urdf:
  > rosrun xacro xacro.py `rospack find sr_description`/robots/arm_and_hand_motor.urdf.xacro -o /tmp/arm_and_hand_motor.urdf
  > rosrun collada_urdf urdf_to_collada /tmp/arm_and_hand_motor.urdf /tmp/arm_and_hand_motor.dae

3. created the xml scene file arm_and_hand_motor.xml

4. you can view it in openrave:
 > roscd openrave/bin
 > export PYTHONPATH=$PYTHONPATH:`./openrave-config --python-dir`
 > ./openrave.py
load from menu.

5. Generated 3DDirection IK:
 > ./openrave.py --database inversekinematics --robot=`rospack find sr_grasp_moving_object`/openrave/arm_and_hand_motor.xml --iktype=Direction3D


=====
links:
http://answers.ros.org/question/12846/could-ik-solver-called-by-arm_navigation-auto-generated-file-work-for-5-dof-manipulator/
http://openrave.org/docs/latest_stable/openravepy/ikfast/
