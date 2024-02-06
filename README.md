# 一个ROS1的发布-订阅方法的问题/A Bug from ROS1 publisher-subscriber method

## 问题与猜测
在进行实验时，发现一个节点订阅到的消息和使用`rostopic echo`指令查看到的消息之间有时延。时延现象随着实验时间的增加而增加，直到该误差已经无法忽略。

## 测试
发现错误后，我们认为可能是接收函数出现了意料之外的问题。因此根据原代码编辑了简单的仿真模块。<br>
<br>
**Publisher：**
```python
import rospy
from std_msgs.msg import String

def talker():
  pub = rospy.Publisher('/talker', String, queue_size=1)
  rate = rospy.Rate(1)
  i=10000
  while not rospy.is_shutdown():
    i+=1
    pub.publish(str(i))
    rate.sleep()
if __name__ == '__main__':
  rospy.init_node('publisher')
  talker()

```
<br>

**Subscriber：**
```python
import rospy
from std_msgs.msg import String

def Callback(msg):
  print(msg.data)

def listener():
  rate = rospy.Rate(0.5)
  while not rospy.is_shutdown():
    rospy.Subscriber('/talker', String, Callback)
    rate.sleep()

if __name__ == '__main__':
  rospy.init_node('listener')
  listener()
```
发现输出与想象中的不一致：
<br>
**实际订阅的：**

```
10005
10006
10006
10007
10007
10007
10008
10008
10008
10008
10009
10009
10009
10009
10009

```
**期望订阅的：**
```
10005
10006
10007
10008
10009
```
<br>
