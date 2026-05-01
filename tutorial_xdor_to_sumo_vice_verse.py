'''
##############################################################
# Created Date: Friday, May 1st 2026
# Contact Info: luoxiangyong01@gmail.com
# Author/Copyright: Mr. Xiangyong Luo
##############################################################
'''

from pathlib import Path
import pyopendrive

if __name__ == "__main__":

    path_xodr = "./datasets/xdor_sumo/chatt.xodr"
    path_sumo = "./datasets/xdor_sumo/chatt.net.xml"

    # Load the OpenDRIVE file
    net_xdor = pyopendrive.OpenDriveMap(path_xodr)

    # convert to SUMO format
    net_sumo = pyopendrive.xodr_to_net_xml(
        xodr_file=Path(path_xodr).absolute(),
        net_file=Path(path_sumo).absolute(),)

    pyopendrive.xodr_from_net_xml(
        net_file=Path(path_sumo).absolute(),
        xodr_file="./datasets/xdor_sumo/chatt_1.xodr")

    pyopendrive.xodr_from_net_xml(
        net = net_sumo,
        xodr_file="./datasets/xdor_sumo/chatt_1.xodr")
