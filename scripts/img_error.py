#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import cv2
import time
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ImageMsg


class ImageError():
    def __init__(self):

        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("decompressed_image", ImageMsg, self.img_callback)

        self.historic = True
        img_compressor_historic = "/img_compressor_v2/historic"
        if rospy.has_param(img_compressor_historic):
            self.historic = rospy.get_param(img_compressor_historic)

        self.grayscale = True
        img_compressor_grayscale = "/img_compressor_v2/grayscale"
        if rospy.has_param(img_compressor_grayscale):
            self.grayscale = rospy.get_param(img_compressor_grayscale)

        self.algoritmo = "DEBT"
        img_compressor_type = "/img_compressor_v2/type"
        if rospy.has_param(img_compressor_type):
            self.algoritmo = rospy.get_param(img_compressor_type)

        if self.algoritmo == "SPIHT":
            if self.grayscale == True:
                self.formato_imagen = "pgm"
            else:
                self.formato_imagen = "ppm"
            self.formato_compresion = "ims"

        elif self.algoritmo == "DEBT":
            self.formato_imagen = "pgm"
            self.formato_compresion = "dbt"

        elif self.algoritmo == "JPEG2000":
            self.formato_imagen = "jpg"
            self.formato_compresion = "jp2"


    def img_callback(self, img_data):
        rospy.loginfo("Imatge rebuda!")

        script_dir = os.path.dirname(os.path.abspath(__file__)) 

        if (self.historic == True):
            img_number = int(len(os.listdir(script_dir + "/compressor_original_image/{}".format(self.algoritmo))))
            RUTA_IMAGEN_A = script_dir + "/compressor_original_image/{}/img{}.{}".format(self.algoritmo, img_number - 1, self.formato_imagen)
            img_number = int(len(os.listdir(script_dir + "/joiner_decompressed_image/{}".format(self.algoritmo))))
            RUTA_IMAGEN_B = script_dir + "/joiner_decompressed_image/{}/img{}.{}".format(self.algoritmo, img_number - 1, self.formato_imagen)
            img_number = int(len(os.listdir(script_dir + "/compressor_compressed_image/{}".format(self.algoritmo))))
            RUTA_IMAGEN_C = script_dir + "/compressor_compressed_image/{}/img{}.{}".format(self.algoritmo, img_number - 1, self.formato_compresion)

        else:
            RUTA_IMAGEN_A = script_dir + "/compressor_original_image/{}/img.{}".format(self.algoritmo, self.formato_imagen)
            RUTA_IMAGEN_B = script_dir + "/joiner_decompressed_image/{}/img.{}".format(self.algoritmo, self.formato_imagen)
            RUTA_IMAGEN_C = script_dir + "/compressor_compressed_image/{}/img.{}".format(self.algoritmo, self.formato_compresion)
 
        if not os.path.exists(RUTA_IMAGEN_A):
            rospy.loginfo("Revisar ruta de la imagen original")
        if not os.path.exists(RUTA_IMAGEN_B):
            rospy.loginfo("Revisar ruta de la imagen descomprimida")
        if not os.path.exists(RUTA_IMAGEN_C):
            rospy.loginfo("Revisar ruta de la imagen comprimida")    


        def mse(imageA, imageB):
            # Calcula el error cuadrático medio entre dos imágenes
            err = np.sum((imageA.astype("float") - imageB.astype("float")) ** 2)
            err /= float(imageA.shape[0] * imageA.shape[1])
            return err

        # Lee las dos imágenes en escala de grises
        image1 = cv2.imread(RUTA_IMAGEN_A, cv2.IMREAD_GRAYSCALE)
        image2 = cv2.imread(RUTA_IMAGEN_B, cv2.IMREAD_GRAYSCALE)

        # Asegúrate de que las imágenes tengan las mismas dimensiones
        if image1.shape != image2.shape:
            rospy.loginfo("Las imágenes deben tener las mismas dimensiones.")
            exit()

        # Calcula el MSE
        error = mse(image1, image2)
        rospy.loginfo("Error Cuadrático Medio (MSE): {:.2f}".format(error))

        # Calcula la Raíz Cuadrada del MSE (RMSE)
        rmse = np.sqrt(error)
        rospy.loginfo("Raíz Cuadrada del Error Cuadrático Medio (RMSE): {:.2f}".format(rmse))

        # Calcular tamaño de la imagen comprimida
        compressed_size = float(os.path.getsize(RUTA_IMAGEN_C))
        units = "B"

        if (compressed_size >= 1024):
            compressed_size /= 1024
            units = "kB"
            if (compressed_size >= 1024):
                compressed_size /= 1024
                units = "MB"
        rospy.loginfo("El tamaño de la imagen comprimida es de : {:.2f} {}".format(compressed_size, units))


def main():
    # Inicializar el nodo
    rospy.init_node('img_error', anonymous=True)

    imgerror = ImageError()

    print("[{:.0f}] Image error node started".format(time.time()))

    try:
        rospy.spin()
    except KeyboardInterrupt as e:
        print("[{:.0f}] Detected KeyboardInterrupt".format(time.time()))
    finally:
        rospy.signal_shutdown("KeyboardInterrupt detected")

    print("[{:.0f}] Image error node ended".format(time.time()))


if __name__ == "__main__":
    main()