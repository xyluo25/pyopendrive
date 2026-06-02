# -*- coding:utf-8 -*-
##############################################################
# Created Date: Thursday, May 14th 2026
# Contact Info: luoxiangyong01@gmail.com
# Author/Copyright: Mr. Xiangyong Luo
##############################################################


import pyopendrive as odr


if __name__ == "__main__":
    # Keep this script alive so the browser can keep calling the local API.
    odr.xodr_web_viewer(port=0, block=True)
