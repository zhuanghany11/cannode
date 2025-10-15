将sa_msgs.tar解压之后，放到colcon build编译生成的install文件夹中，然后source后再次进行colcon build编译，以使之生效。
主要原因在与sa_msgs版本当前在x86电脑上不完全适配，因此直接拿到编译好的文件进行测试，用于判断代码编译成功与否。

编译后的代码结构应该为：
/build
/log
/install
 |------cannode
 |------sa_msgs
 others
/src/
 |-------cannode
sa_msgs.tar
