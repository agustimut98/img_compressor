#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def publish_image():

    # Inicializar el nodo
    rospy.init_node('image_publisher', anonymous=True)
    
    # Inicializar el publicador
    pub = rospy.Publisher('original_image', Image, queue_size=10)
    
    # Instanciar el puente entre OpenCV y ROS
    bridge = CvBridge()
    
    # Leer la imagen usando OpenCV
    # Ruta de la imatge inicial /home/agusti/Descargas/TFM/ICVV-2017_DEBT/proves_anteriors/38.jpg
    img = cv2.imread('/home/agusti/Descargas/img_pool.pgm')
    # img = cv2.imread('/home/agusti/Descargas/TFM/ICVV-2017_DEBT/proves_anteriors/38.jpg') 

    # Convertir la imagen de OpenCV a un mensaje de ROS
    img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
    
    rate = rospy.Rate(0.2) # Publicar cada 5 segundos

    while not rospy.is_shutdown():

        # Publicar el mensaje
        pub.publish(img_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass
