# -*- coding: utf-8 -*-
#!/usr/bin/env python3

from aimooe_pub.aim import package_path
from aimooe_pub.aim import tracker

def main(args=None):
    
    obj = tracker(package_path+'/Aimtools/')

    tool_name = ""
    while len(tool_name) <= 1:
        tool_name = input("输入工具名称:")
    obj.calibrate_4Ptool(tool_name)

if __name__ == '__main__':

    main()