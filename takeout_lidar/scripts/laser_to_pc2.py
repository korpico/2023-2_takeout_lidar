#!/usr/bin/env python3

## 관련된 라이브러리들 가져오기
import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import math

## laserscan_to_pointcloud 노드 초기화
rospy.init_node("laserscan_to_pointcloud")
## LaserProjection 객체 생성
lp = lg.LaserProjection()
## Publisher 설정
pc_pub = rospy.Publisher("/cloud", PointCloud2, queue_size=1)

def scan_cb(msg):
    ## Laser 메시지 객체 생성 
    pc2_msg = lp.projectLaser(msg)
    ## 인덱스로 접근할 수 있도록 변환
    pc_pub.publish(pc2_msg)
    ## 평균 z 값과 중간 포인트를 계산하여 출력
    point_generator = pc2.read_points(pc2_msg)
    sum = 0.0
    num = 0
    for point in point_generator:
        if not math.isnan(point[2]):
            sum += point[2]
            num += 1

    ## 평균 z값
    print(str(sum/num))

    point_list = pc2.read_points_list(pc2_msg)
    ## PointCloud2 데이터의 중간 포인트의 x값
    print(point_list[int(len(point_list) / 2)].x)

##  서브스크라이버
rospy.Subscriber("/scan", LaserScan, scan_cb, queue_size=1)
rospy.spin()