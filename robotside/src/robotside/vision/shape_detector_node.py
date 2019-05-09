#!/usr/bin/env python
import rospy
from benthic_species_detector import BenthicSpeciesDetector
from std_msgs.msg import UInt8, UInt8MultiArray, MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import Image


class ShapeDetector:
    def __init__(self):
        self.species_detector = BenthicSpeciesDetector(rect_similarity_alpha=0.1, rect_enclosure_alpha=0.1,
                                                       shape_alpha=0.1, corner_min_distance=10)
        self.species_pub = rospy.Publisher('species', UInt8MultiArray, queue_size=5)

    def detect_and_publish(self, image):
        shapes_list = self.species_detector.process(image, image_scaling_factor=0.6, debug=False)
        species_counts = [x * 0 for x in range(0, 4)]
        for shape_info in shapes_list:
            shape_type = shape_info[1][0]
            if shape_type == species_detector.ShapeType.triangle:
                species_counts[0] += 1
            elif shape_type == species_detector.ShapeType.rectangle:
                species_counts[1] += 1
            elif shape_type == species_detector.ShapeType.square:
                species_counts[2] += 1
            elif shape_type == species_detector.ShapeType.circle:
                species_counts[3] += 1
        species_counts = list(map(lambda count: UInt8(count), species_counts))
        species_msg = UInt8MultiArray(data=species_counts)
        species_pub.publish(species_msg)


if __name__ == "__main__":
    rospy.init_node("ShapeDetector")

    shape_detector = ShapeDetector()

    rospy.Subscriber("stereo_cam/left", Image, shape_detector.detect_and_publish)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
