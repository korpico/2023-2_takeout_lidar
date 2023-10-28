#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
from geometry_msgs.msg import Point32
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

def velodyne_callback(msg):
    # PointCloud2 메시지에서 데이터 읽기
    pc_data = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

    # 필터링 조건 설정
    filtered_data = [point for point in pc_data if ((0 <= point[0] <= 4) & (-2 <= point[1] <= 2))]

    # 필터링된 데이터에서 xyz 좌표만 추출하여 클러스터링을 수행
    points = np.array([[point[0], point[1], point[2]] for point in filtered_data])
    clustering = DBSCAN(eps=0.2, min_samples=10).fit(points)

    # 클러스터링 결과를 시각화하기 위한 Marker 생성
    marker_array = create_cluster_markers(clustering.labels_, filtered_data)

    # 클러스터링 결과와 시각화 정보를 퍼블리시
    pub.publish(create_clustered_point_cloud(filtered_data, clustering.labels_, msg.header))
    marker_pub.publish(marker_array)

def create_cluster_markers(labels, data):
    marker_array = MarkerArray()
    unique_labels = set(labels)
    color_id = 0

    for label in unique_labels:
        if label == -1:  # 노이즈 포인트는 건너뜀
            continue

        cluster_points = [data[i] for i in range(len(data)) if labels[i] == label]

        if len(cluster_points) < 5:  # 작은 클러스터는 무시
            continue

        # 클러스터 중심 계산
        cluster_center = np.mean(cluster_points, axis=0)[:3]

        # 클러스터를 시각화하는 Marker 생성
        marker = Marker()
        marker.header.frame_id = "laser_frame"
        marker.type = Marker.SPHERE
        marker.id = color_id
        marker.pose.position.x = cluster_center[0]
        marker.pose.position.y = cluster_center[1]
        marker.pose.position.z = cluster_center[2]
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = color_id / len(unique_labels)
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker_array.markers.append(marker)
        color_id += 1

    return marker_array

def create_clustered_point_cloud(data, labels, header):
    cluster_cloud = PointCloud2()
    cluster_cloud.header = header

    # 클러스터링 결과에 따라 width 값을 설정
    cluster_cloud.width = len(labels)

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
    cluster_cloud.fields = fields

    cluster_points = []
    for i in range(len(data)):
        x, y, z = data[i]
        cluster_points.extend([x, y, z])

    # 실제 데이터 크기와 point_step 값을 설정
    cluster_cloud.data = np.array(cluster_points, dtype=np.float32).tobytes()
    cluster_cloud.point_step = 12  # 각 포인트의 크기 (x, y, z, 각각 4바이트)

    # height 값은 1로 설정
    cluster_cloud.height = 1

    return cluster_cloud


if __name__ == "__main__":
    rospy.init_node("point_cloud_processing_node")
    
    # /roi_cloud 토픽을 구독하여 데이터 수신
    rospy.Subscriber("/cloud", PointCloud2, velodyne_callback, queue_size=10)
    
    # 클러스터링 결과를 퍼블리시
    pub = rospy.Publisher("/clustered_cloud", PointCloud2, queue_size=10)
    
    # 클러스터링 결과를 시각화하기 위한 Marker를 퍼블리시
    marker_pub = rospy.Publisher("/cluster_markers", MarkerArray, queue_size=10)
    
    rospy.spin()
