'''
##############################################################
# Created Date: Friday, May 1st 2026
# Contact Info: luoxiangyong01@gmail.com
# Author/Copyright: Mr. Xiangyong Luo
##############################################################
'''

from pathlib import Path
import pyopendrive as odr

if __name__ == "__main__":

    path_xodr = "./datasets/xdor_sumo/chatt.xodr"
    path_sumo = "./datasets/xdor_sumo/chatt.net.xml"

    # Load the OpenDRIVE file
    net_xdor = odr.readXodr(path_xodr)

    # convert to SUMO format
    net_sumo = odr.xodr_to_net_xml(
        xodr_file=Path(path_xodr).absolute(),
        net_file=Path(path_sumo).absolute(),)

    odr.xodr_from_net_xml(
        net_file=Path(path_sumo).absolute(),
        xodr_file="./datasets/xdor_sumo/chatt_1.xodr")

    odr.xodr_from_net_xml(
        net = net_sumo,
        xodr_file="./datasets/xdor_sumo/chatt_1.xodr")
