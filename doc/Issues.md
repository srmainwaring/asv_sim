# Issues

<details>
  <summary>1. package format version in package.xml</summary>

The `package.xml` for `asv_common` must use `<package format="1">` which supports the `<run_depend>` tag.

If `<package format="2">` is used then `roslaunch` will issue the warning below.

```bash
$ roslaunch asv_gazebo asv.launch
...
WARNING: Metapackage "asv_common" should not have other dependencies besides a buildtool_depend on catkin and run_depends.
```
</details>

<details>
  <summary>2. spawn_model in gazebo_ros</summary>

The tutorial [Using roslaunch to start Gazebo, world files and URDF models](http://gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros) describes how to spawn a URDF model inside a running gazebo session. The command is:

```bash
rosrun gazebo_ros spawn_model -file `rospack find asv_description`/urdf/asv.urdf -urdf -model asv
```

The command fails when using python 3 with ros melodic resuting in this error:

```bash
$ rosrun gazebo_ros spawn_model -file `rospack find asv_description`/urdf/asv.urdf -urdf -model asv
[INFO] [1539687860.202245, 3577.896000]: Loading model XML from file ...
[INFO] [1539687860.209548, 3577.903000]: Waiting for service /gazebo/spawn_urdf_model
[INFO] [1539687860.219891, 3577.914000]: Calling service /gazebo/spawn_urdf_model
Traceback (most recent call last):
  File "/opt/ros/melodic/lib/gazebo_ros/spawn_model", line 232, in <module>
    exit_code = sm.run()
  File "/opt/ros/melodic/lib/gazebo_ros/spawn_model", line 178, in run
    self.args.gazebo_namespace)
  File "/opt/ros/melodic/lib/python3.6/site-packages/gazebo_ros/gazebo_interface.py", line 32, in spawn_urdf_model_client
    resp = spawn_urdf_model(model_name, model_xml, robot_namespace, initial_pose, reference_frame)
  File "/opt/ros/melodic/lib/python3.6/site-packages/rospy/impl/tcpros_service.py", line 439, in __call__
    return self.call(*args, **kwds)
  File "/opt/ros/melodic/lib/python3.6/site-packages/rospy/impl/tcpros_service.py", line 516, in call
    transport.send_message(request, self.seq)
  File "/opt/ros/melodic/lib/python3.6/site-packages/rospy/impl/tcpros_base.py", line 668, in send_message
    serialize_message(self.write_buff, seq, msg)
  File "/opt/ros/melodic/lib/python3.6/site-packages/rospy/msg.py", line 152, in serialize_message
    msg.serialize(b)
  File "/opt/ros/melodic/lib/python3.6/site-packages/gazebo_msgs/srv/_SpawnModel.py", line 103, in serialize
    _x = _x.encode('utf-8')
AttributeError: 'bytes' object has no attribute 'encode'
Exception in thread Thread-3:
Traceback (most recent call last):
  File "/usr/local/Cellar/python/3.6.5_1/Frameworks/Python.framework/Versions/3.6/lib/python3.6/threading.py", line 916, in _bootstrap_inner
    self.run()
  File "/usr/local/Cellar/python/3.6.5_1/Frameworks/Python.framework/Versions/3.6/lib/python3.6/threading.py", line 864, in run
    self._target(*self._args, **self._kwargs)
  File "/opt/ros/melodic/lib/python3.6/site-packages/rospy/impl/tcpros_base.py", line 154, in run
    (client_sock, client_addr) = self.server_sock.accept()
  File "/usr/local/Cellar/python/3.6.5_1/Frameworks/Python.framework/Versions/3.6/lib/python3.6/socket.py", line 205, in accept
    fd, addr = self._accept()
ConnectionAbortedError: [Errno 53] Software caused connection abort
```

The problem is caused by this line in `gazebo_ros_pkgs/gazebo_ros/scripts/spawn_model`

```python
        # Encode xml object back into string for service call
        model_xml = xml.etree.ElementTree.tostring(xml_parsed)
```

In python 3 the `tostring` method returns a binary encoded string, which then goes on to cause the seralize method to fail in the generated code for the service call. To fix this change the xml encoding to:

```python
        model_xml = xml.etree.ElementTree.tostring(xml_parsed).decode('utf-8')
```
</details>

<details>
  <summary>3. translating an object gives a malloc: *** error</summary>

```bash
gzserver(6214,0x700000104000) malloc: *** error for object 0x7f87f627c210: pointer being freed was not allocated
*** set a breakpoint in malloc_error_break to debug
/opt/ros/melodic/lib/gazebo_ros/gzserver: line 41:  6214 Abort trap: 6           GAZEBO_MASTER_URI="$desired_master_uri" GAZEBO_MODEL_DATABASE_URI="$desired_model_database_uri" gzserver $final
[gazebo-2] process has died [pid 6140, exit code 134, cmd /opt/ros/melodic/lib/gazebo_ros/gzserver --verbose -e ode /Users/rhys/Code/ros/asv_ws/src/asv_common/asv_gazebo/worlds/asv.world __name:=gazebo __log:=/Users/rhys/.ros/log/96c4d540-06c5-11e9-90d2-0017f203fbe8/gazebo-2.log].
log file: /Users/rhys/.ros/log/96c4d540-06c5-11e9-90d2-0017f203fbe8/gazebo-2*.log
```

</details>
