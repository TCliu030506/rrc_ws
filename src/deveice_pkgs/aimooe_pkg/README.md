# 元功能包介绍
Aimooe光定位仪模块,采用python编写

## aimooe_sdk
定义了Aimooe发送的消息格式,提供了Aimooe驱动模块

## aimooe_pub
发布Aimooe读取到的坐标消息,也提供python模块:aimooe_client读取某个坐标消息

# 使用方法
1. 使用aimooe_pub功能包的节点设置Aimooe的工具
2. 运行aimooe_pub功能包的launch文件发布光定位坐标
3. 监听aimooe_sdk定义的话题,或者使用python模块的client类直接读取光定位数据(from aimooe_pub.aimooe_client import Subscriber as Aim)