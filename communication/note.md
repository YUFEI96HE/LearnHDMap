1. HandleCCU需要提供接口给其他类来直接发送消息给CCU
    HandleADT，HandleAPP会给控制信息
    除此之外，HandleADT,HandleAPP都需要添加Send的接口，用作消息回复[Done]

2. 缺少建图任务执行脚本[Done]

3. 把ADT发送的左上角右下角的数据通过topic发送，频率1，数据结构 geometry_msgs/PolygonStamped
    sweep_rectangle   作业区域话题  
    自定义消息数据结构  把ADT0x04的任务信息用这个数据结构发布
    现在先用uint8_t的数据结构来发布，之后改成string或intx格式方便其他模块订阅使用
    所有数据发布在 adam_msgs/vehicle_cmd[Done]

4. 注意ControlRawData字段 来自ADT和CCU的不一致 ACU需要透传ADT给CCU的控制信息[Done]

5. 有一个问题，ADT APP给ACU下发的任务数据，应该有一个变量来控制此时用哪一方的数据作为ROS消息发布。

6. 12月1日配合培楠王灿实车测试本模块[Done]

7. 在开启节点的时，长时间没有建立连接会断开相应的连接请求。断开连接后设置重连。稳定性怎么提高

8. SendCCU SendADT[Done] 

9. ADT任务调度 扫地 洒水类型反了[Done]

10. 
