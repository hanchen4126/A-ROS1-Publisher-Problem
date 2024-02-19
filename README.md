# 一个ROS1的发布-订阅方法的问题

## 问题
在进行实验时，发现一个节点订阅到的消息和使用`rostopic echo`指令查看到的消息之间有时延。时延现象随着实验时间的增加而增加，直到该误差已经无法忽略。

## 测试
为了定位问题，我们尝试对时延现象进行复现。发现在回调函数中直接输出接受到的消息时会出现大块大块的更新现象。因此我们认为可能是ROS的接收功能出现异常。为了方便观测异常，我们将可能出现问题的代码简化。<br>
<br>
**发布者：**
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

**订阅者：**
```python
import rospy
from std_msgs.msg import String

def Callback(msg):
  print(msg.data)

def listener():
  rate = rospy.Rate(1)
  while not rospy.is_shutdown():
    rospy.Subscriber('/talker', String, Callback)
    rate.sleep()

if __name__ == '__main__':
  rospy.init_node('listener')
  listener()
```

<br>可以观测到实际订阅的消息输出与期望不同，这也解释了大块更新现象。<br>

**实际：**
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

**期望：**

```
10005
10006
10007
10008
10009
```

<br>实际的订阅消息似乎呈现一种规律。每次从消息处获取新的信息时，回调函数似乎会被多调用一次。我们知道rospy订阅新的话题时会在节点进程中开启一个线程处理该订阅者。那么这个现象是rospy在节点进程的主线程运行到订阅者的构造函数时开启了新的线程，还是rospy管理订阅者的构造函数线程再一次进行了构造，亦或者是别的原因导致了回调函数被多次调用的出现呢？为了进一步确认订阅者在ROS1的作用机制，我们进行了进一步测试。<br>

**多开线程？**
<br>Ubuntu可以通过进程PID查询线程情况。
```
$ rosnode info listener
...
Pid:37581
...

$ top -p 37581
#键盘中输入shift+h查询所有线程#
PID USER      PR  NI    VIRT    RES    SHR S  %CPU 
  37581 ***       20   0  508628  41360  14408 S   0.0 
  37593 ***       20   0  508628  41360  14408 S   0.0 
  37594 ***       20   0  508628  41360  14408 S   0.0 
  37596 ***       20   0  508628  41360  14408 S   0.0 
  37599 ***       20   0  508628  41360  14408 S   0.0 
  37602 ***       20   0  508628  41360  14408 S   0.0
```
随着单次订阅到的消息越来越多，线程总数并没有增加。

**多次构造？**
<br>修改订阅者代码
```python
import rospy
import time
from std_msgs.msg import String

def Callback1(msg):
  print(msg.data + 'cb1')

def Callback2(msg):
  print(msg.data + 'cb2')

def Callback3(msg):
  print(msg.data + 'cb3')

def Callback4(msg):
  print(msg.data + 'cb4')

def listener():
  rate = rospy.Rate(1)
  count = 0
  while not rospy.is_shutdown():
    s_time = time.time()
    if count == 0:
      rospy.Subscriber('/talker', String, Callback1)
    elif count == 1:
      rospy.Subscriber('/talker', String, Callback2)
    elif count == 2:
      rospy.Subscriber('/talker', String, Callback3)
    else:
      rospy.Subscriber('/talker', String, Callback4)
    count += 1
    rate.sleep()
    print(time.time()-s_time)

if __name__ == '__main__':
  rospy.init_node('listener')
  listener()
```
运行结果为：
```
10003cb1
1.0013244152069092
10004cb1
10004cb2
0.9998748302459717
10005cb1
10005cb2
10005cb3
0.9999167919158936
10006cb1
10006cb2
10006cb3
10006cb4
0.9998745918273926
10007cb1
10007cb2
10007cb3
10007cb4
10007cb4
0.9999985694885254
10008cb1
10008cb2
10008cb3
10008cb4
10008cb4
10008cb4
0.9999454021453857
```
根据以上结果，可以猜测：主线程每次循环都将构造一个订阅者到订阅者线程。订阅者线程将所有订阅者的回调函数存入一个队列，并使用同一个消息运行运行队列中的回调函数。但是，这个猜测并不能解释时延的现象。
