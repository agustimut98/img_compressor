#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import cv2
import rospy
import glymur
import subprocess
import numpy as np
from PIL import Image


def run_imshrinker_compression(input_file, output_file, script_dir, bpp,):
    bpp = "c" + str(bpp)

    # Construye el comando como una lista de strings 
    command = [os.path.join(script_dir, "imshrinker"), bpp , input_file, output_file]
    
    # Ejecuta el comando. Comadno de ejemplo de REDAME.md: ./imshrinker  c0.7 ../test-data/new425.ppm ../test-data/Cnew425ppm.ims
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()


def run_imshrinker_decompression(input_file, output_file, script_dir):

    # Construye el comando como una lista de strings 
    command = [os.path.join(script_dir, "imshrinker"), "d", input_file, output_file]
    
    # Ejecuta el comando. Ejemplo de REDME.md: ./imshrinker  d  ../test-data/Cnew425ppm.ims ../test-data/Dnew425ppm.ppm
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()


def run_debter_compression(input_file, output_file, script_dir, quality, log, method, nbands, transform):
    quality = str(quality)
    log = str(log)
    method = str(method)
    nbands = str(nbands)
    transform = str(transform)

    # Construye el comando como una lista de strings
    command = [os.path.join(script_dir, "debter"), "-t", transform, "-r", method, "-q", quality, "-f", log]

    # Abre los archivos de entrada y salida
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile, open(os.devnull, 'w') as DEVNULL:

        # Ejecuta el comando. Ejemplo de help.txt: ./debter -t cdf-9/7 -r 2 -q 4 -f 1 < lena.pgm > lena.dbt 2> log.txt
        process = subprocess.Popen(command, stdin=infile, stdout=outfile, stderr=DEVNULL)
        process.communicate()


def run_debter_decompression(input_file, output_file, script_dir):

    # Construye el comando como una lista de strings
    command = [os.path.join(script_dir, "debter"), "-d"]
    
    # Abre los archivos de entrada y salida
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile, open(os.devnull, 'w') as DEVNULL:

        # Ejecuta el comando. Ejemplo de help.txt: ./debter -d < lena.dbt > dlena.pgm
        process = subprocess.Popen(command, stdin=infile, stdout=outfile, stderr=DEVNULL)
        process.communicate()


def mse(imageA, imageB):
    # Calcula el error cuadrático medio entre dos imágenes
    err = np.sum((imageA.astype("float") - imageB.astype("float")) ** 2)
    err /= float(imageA.shape[0] * imageA.shape[1])
    return err


def main():
    rospy.init_node('img_calibrator', anonymous=True)
    script_dir = os.path.dirname(os.path.abspath(__file__))

    compression_params = {
        # Valors calibracio piscina
        # 'SPIHT': [0.1], 'DEBT': [10], 'JPEG2000': [400]

        # Valors calibracio bagfile
        # 'SPIHT': [0.16], 'DEBT': [10], 'JPEG2000': [185]


        'SPIHT': [0.025, 0.05, 0.1,0.14,0.15,0.155,0.16, 0.17, 0.18, 0.19,0.2, 0.4, 0.8], 
        'DEBT': [ 0.5, 1, 2, 4, 8, 10,  16 ],
        'JPEG2000': [15.625, 31.25, 62.5, 125,150, 175,185, 200, 225,250,285, 300, 350, 400, 450, 500]
    }

        
    for key in compression_params.keys():
        cont = 0

        for element in compression_params[key]:
                
            if key == "SPIHT":
                formato_imagen = "pgm"
                formato_compresion = "ims"

                RUTA_IMAGEN_A = "/home/agusti/Descargas/img0_bagfile.pgm"
                RUTA_IMAGEN_B = "/home/agusti/Descargas/test/img0_bagfile.pgm"
                RUTA_IMAGEN_C = "/home/agusti/Descargas/test/img0_bagfile.ims"

                #RUTA_IMAGEN_A = script_dir + "/compressor_original_image/{}/img0.{}".format(key, formato_imagen)
                #RUTA_IMAGEN_B = script_dir + "/joiner_decompressed_image/{}/img{}.{}".format(key, cont, formato_imagen)
                #RUTA_IMAGEN_C = script_dir + "/compressor_compressed_image/{}/img{}.{}".format(key, cont, formato_compresion)

                if not os.path.exists(RUTA_IMAGEN_A):
                    rospy.loginfo("Revisar ruta de la imagen original")

                run_imshrinker_compression(RUTA_IMAGEN_A, RUTA_IMAGEN_C, script_dir, element)
                run_imshrinker_decompression(RUTA_IMAGEN_C, RUTA_IMAGEN_B, script_dir)

                # Lee las dos imágenes en escala de grises
                image1 = cv2.imread(RUTA_IMAGEN_A, cv2.IMREAD_GRAYSCALE)
                image2 = cv2.imread(RUTA_IMAGEN_B, cv2.IMREAD_GRAYSCALE)

                # Asegúrate de que las imágenes tengan las mismas dimensiones
                if image1.shape != image2.shape:
                    rospy.loginfo("Las imágenes deben tener las mismas dimensiones.")
                    exit()

            
            elif key == "DEBT":
                formato_imagen = "pgm"
                formato_compresion = "dbt"

                RUTA_IMAGEN_A = "/home/agusti/Descargas/img0_bagfile.pgm"
                RUTA_IMAGEN_B = "/home/agusti/Descargas/test/img0_bagfile.pgm"
                RUTA_IMAGEN_C = "/home/agusti/Descargas/test/img0_bagfile.dbt"

                #RUTA_IMAGEN_A = script_dir + "/compressor_original_image/{}/img0.{}".format(key, formato_imagen)
                #RUTA_IMAGEN_B = script_dir + "/joiner_decompressed_image/{}/img{}.{}".format(key, cont, formato_imagen)
                #RUTA_IMAGEN_C = script_dir + "/compressor_compressed_image/{}/img{}.{}".format(key, cont, formato_compresion)

                if not os.path.exists(RUTA_IMAGEN_A):
                    rospy.loginfo("Revisar ruta de la imagen original") 

                run_debter_compression(RUTA_IMAGEN_A, RUTA_IMAGEN_C, script_dir, element, 0, 2, 6, "cdf-9/7")
                run_debter_decompression(RUTA_IMAGEN_C, RUTA_IMAGEN_B, script_dir)

                # Lee las dos imágenes en escala de grises
                image1 = cv2.imread(RUTA_IMAGEN_A, cv2.IMREAD_GRAYSCALE)
                image2 = cv2.imread(RUTA_IMAGEN_B, cv2.IMREAD_GRAYSCALE)

                # Asegúrate de que las imágenes tengan las mismas dimensiones
                if image1.shape != image2.shape:
                    rospy.loginfo("Las imágenes deben tener las mismas dimensiones.")
                    exit()


            elif key == "JPEG2000":
                formato_imagen = "jpg"
                formato_compresion = "jp2"

                RUTA_IMAGEN_A = "/home/agusti/Descargas/img0_bagfile.pgm"
                RUTA_IMAGEN_B = "/home/agusti/Descargas/test/img0_bagfile.pgm"
                RUTA_IMAGEN_C = "/home/agusti/Descargas/test/img0_bagfile.jp2"

                #RUTA_IMAGEN_A = script_dir + "/compressor_original_image/{}/img0.{}".format(key, formato_imagen)
                #RUTA_IMAGEN_B = script_dir + "/joiner_decompressed_image/{}/img{}.{}".format(key, cont, formato_imagen)
                #RUTA_IMAGEN_C = script_dir + "/compressor_compressed_image/{}/img{}.{}".format(key, cont, formato_compresion)
                
                if not os.path.exists(RUTA_IMAGEN_A):
                    rospy.loginfo("Revisar ruta de la imagen original")

                cv_image = cv2.imread(RUTA_IMAGEN_A)
                # Comprimir imagen en formato .jp2
                glymur.Jp2k(RUTA_IMAGEN_C, data=cv_image, cratios=[element])
                # Executar descompresió amb JPEG2000
                Image.fromarray(glymur.Jp2k(RUTA_IMAGEN_C)[:]).save(RUTA_IMAGEN_B)

                # Lee las dos imágenes en escala de grises
                image1 = cv2.imread(RUTA_IMAGEN_A, cv2.IMREAD_GRAYSCALE)
                image2 = cv2.imread(RUTA_IMAGEN_B, cv2.IMREAD_GRAYSCALE)

                # Asegúrate de que las imágenes tengan las mismas dimensiones
                if image1.shape != image2.shape:
                    rospy.loginfo("Las imágenes deben tener las mismas dimensiones.")
                    exit()

            rospy.loginfo("Algoritmo usado: {}".format(key))
            rospy.loginfo("Parametro de compresion: {}".format(element))

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
            rospy.loginfo("El tamaño de la imagen comprimida es de : {:.2f} {} \n".format(compressed_size, units))

            with open(script_dir +'/testbench.txt', 'a') as archivo:
                archivo.write("algoritmo: {}   ".format(key))
                archivo.write("parameter: {}   ".format(element))

                archivo.write("MSE: {:.2f}   ".format(error))
                archivo.write("RMSE: {:.2f}   ".format(rmse))
                archivo.write("Size: {:.2f} {}   \n \n".format(compressed_size, units))

            cont += 1
        
        with open('testbench.txt', 'a') as archivo:
            archivo.write("\n \n")


if __name__ == "__main__":
    main()