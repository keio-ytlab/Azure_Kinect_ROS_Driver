#!/usr/bin/env python

import rospy
import numpy as np
import ros_numpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image


class PcdImageConverter():
    def __init__(self):
        self.sub_pcd_ = rospy.Subscriber("~input_pcd", PointCloud2, self.cb_pcd, queue_size=1)
        self.pub_image_ = rospy.Publisher("~output_image", Image, queue_size=1)

    def cb_pcd(self, _pcd_msg):
        image = self.generate_image(_pcd_msg)
        image.header = _pcd_msg.header
        if image is not None:
            self.pub_image_.publish(image)

    def generate_image(self, pcd):
        # start_time = time.time()

        # pcd and image size
        if pcd is not None:
            width = pcd.width
            height = pcd.height
            if height == 1:
                resolution = width
                # NFOV Unbinned
                if resolution == 368640:
                    width = 640
                    height = 576
                # NFOV 2x2 Binned (SW)
                elif resolution == 92160:
                    width = 320
                    height = 288
                # WFOV 2x2 Binned
                elif resolution == 262144:
                    width = 512
                    height = 512
                # WFOV Unbinned
                elif resolution == 1048576:
                    width = 1024
                    height = 1024
                # 720P (same with rgb resolution)
                elif resolution == 230400:
                    width = 640
                    height = 360
                else:
                    rospy.logerr("[Detectron2 ROS] Received Unknown Resolution PCD")
                    rospy.logerr("    You can use generating image from pcd onyl when using Azure Kinect DK.")

            # convert
            array = np.zeros((height, width), dtype=np.float32)

            # save data to arrary
            pcd_original = ros_numpy.numpify(pcd)
            pcd_original = pcd_original.reshape(height, width)

            # PCD with rgb
            if 'rgb' in pcd_original:
                array[:, :] = pcd_original['rgb']
            # PCD without rgb
            else:
                array[:, :] = pcd_original['x'] * 255.0

            # convert color (html color -> rgb)
            data = array.view(np.uint8).reshape(array.shape + (4,))[..., :3]
            image = ros_numpy.msgify(Image, data, encoding='bgr8')
            # finish_time = time.time()
            # print("generate image time : {0}".format(finish_time-start_time))

            return image
        else:
            return None


if __name__ == '__main__':
    rospy.init_node("pcd_image_converter")
    PcdImageConverter()
    rospy.spin()
