#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np

# Inicializar el nodo
rospy.init_node('img_error', anonymous=True)

RUTA_IMAGEN_A = "/home/agusti/catkin_ws/src/img_compressor_v2/scripts/compressor_original_image/SPIHT/img0.pgm"
RUTA_IMAGEN_B = "/home/agusti/catkin_ws/src/img_compressor_v2/scripts/joiner_decompressed_image/SPIHT/img0.pgm"

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