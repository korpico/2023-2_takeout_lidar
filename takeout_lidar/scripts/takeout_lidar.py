#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import numpy as np

def velodyne_callback(msg):
    # PointCloud2 메시지에서 데이터 읽기
    ## x, y, z 속성 필드로 이루어진 리스트
    pc_data = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    ## roi 연산 (현재 X축 : 0 ~ 4m, Y축 : -2 ~ 2m)
    filtered_data = [point for point in pc_data if ((0 <= point[0] <= 4) & (-2 <= point[1] <= 2))]
    ## 메시지 객체 생성 및 퍼블리시
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = msg.header.frame_id  # 원본 메시지와 동일한 프레임 ID 사용
    new_pc_msg = point_cloud2.create_cloud_xyz32(header, filtered_data)

    # 새로운 메시지를 퍼블리시
    pub.publish(new_pc_msg)

if __name__ == "__main__":
    rospy.init_node("point_cloud_passthrough_node")

    ## 서브스크라이버, 퍼블리서 초기화
    # /cloud 토픽을 구독하여 데이터 수신
    rospy.Subscriber("/cloud", PointCloud2, velodyne_callback, queue_size=10)
    # /roi_cloud 토픽으로 PointCloud2 메시지 퍼블리시
    pub = rospy.Publisher("/roi_cloud", PointCloud2, queue_size=10)
    
    rospy.spin()
