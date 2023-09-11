#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import cv2
import rospy
import numpy as np

# Inicializar el nodo
rospy.init_node('img_error', anonymous=True)

algoritmo = "DEBT"
img_compressor_type = "/img_compressor_v2/type"
if rospy.has_param(img_compressor_type):
    algoritmo = rospy.get_param(img_compressor_type)

if algoritmo == "SPIHT":
    formato_imagen = "pgm"
    formato_compresion = "ims"

elif algoritmo == "DEBT":
    formato_imagen = "pgm"
    formato_compresion = "dbt"

elif algoritmo == "JPEG2000":
    formato_imagen = "jpg"
    formato_compresion = "jp2"

script_dir = os.path.dirname(os.path.abspath(__file__)) 

RUTA_IMAGEN_A = script_dir + "/compressor_original_image/{}/img0.{}".format(algoritmo, formato_imagen)
RUTA_IMAGEN_B = script_dir + "/joiner_decompressed_image/{}/img0.{}".format(algoritmo, formato_imagen)
RUTA_IMAGEN_C = script_dir + "/compressor_compressed_image/{}/img0.{}".format(algoritmo, formato_compresion)

if not os.path.exists(RUTA_IMAGEN_A):
    rospy.loginfo("Revisar ruta de la imagen original")
if not os.path.exists(RUTA_IMAGEN_B):
    rospy.loginfo("Revisar ruta de la imagen decomprimida")
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
compressed_size = os.path.getsize(RUTA_IMAGEN_C)
units = "B"

if (compressed_size >= 1024):
    compressed_size /= 1024
    units = "kB"
    if (compressed_size >= 1024):
        compressed_size /= 1024
        units = "MB"
rospy.loginfo("El tamaño de la imagen comprimida es de : {:.2f} {}".format(compressed_size, units))