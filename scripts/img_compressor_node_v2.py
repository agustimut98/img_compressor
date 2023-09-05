#!/usr/bin/env python
# -*- coding: utf-8 -*-

from time import time
import rospy
import cv2 as cv
from sensor_msgs.msg import Image as ImageMsg
from img_compressor.msg import BinarySplit
from cv_bridge import CvBridge
from PIL import Image
import glymur
import subprocess
import os
import time


def run_imshrinker(input_file, output_file, script_dir):

    # Construye el comando como una lista de strings 
    command = [os.path.join(script_dir, "imshrinker"), "c0.9" , input_file, output_file]
    
    # Ejecuta el comando. Comadno de ejemplo de REDAME.md: ./imshrinker  c0.7 ../test-data/new425.ppm ../test-data/Cnew425ppm.ims
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()


def run_debter(input_file, output_file, script_dir):

    # Construye el comando como una lista de strings
    command = [os.path.join(script_dir, "debter"), "-t", "cdf-9/7", "-r", "2", "-q", "4", "-f", "1"]

    # Abre los archivos de entrada y salida
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile, open(os.devnull, 'w') as DEVNULL:

        # Ejecuta el comando. Ejemplo de help.txt: ./debter -t cdf-9/7 -r 2 -q 4 -f 1 < lena.pgm > lena.dbt 2> log.txt
        process = subprocess.Popen(command, stdin=infile, stdout=outfile, stderr=DEVNULL)
        process.communicate()


class ImageCompressor:
    def __init__(self):

        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("original_image", ImageMsg, self.img_callback)
        self.img_pub = rospy.Publisher("compressed_image", BinarySplit, queue_size = 10)

        self.comp_type = "JPEG2000"
        img_compressor_type = "/img_compressor/type"
        if rospy.has_param(img_compressor_type):
            self.comp_type = rospy.get_param(img_compressor_type) 

        self.historic = True
        img_compressor_historic = "/img_compressor/historic"
        if rospy.has_param(img_compressor_historic):
            self.historic = rospy.get_param(img_compressor_historic)

        self.grayscale = False
        img_compressor_grayscale = "/img_compressor/grayscale"
        if rospy.has_param(img_compressor_grayscale):
            self.grayscale = rospy.get_param(img_compressor_grayscale)

        self.comp_ratio = 500
        img_compressor_ratio = "/img_compressor/ratio"
        if rospy.has_param(img_compressor_ratio):
            self.comp_ratio = rospy.get_param(img_compressor_ratio)

        self.img_width = 0
        img_compressor_width = "/img_compressor/width"
        if rospy.has_param(img_compressor_width):
            self.img_width = rospy.get_param(img_compressor_width)

        self.img_height= 0
        img_compressor_height = "/img_compressor/height"
        if rospy.has_param(img_compressor_height):
            self.img_height = rospy.get_param(img_compressor_height)
 

    def img_callback(self, img_data):
        script_dir = os.path.dirname(os.path.abspath(__file__))

        #Establir ruta on guardar la imatge rebuda al topic original_image
        file_path_input = os.path.join(script_dir, "original_image")

        # Crear el directori en cas de no existir
        if not os.path.exists(file_path_input):
            os.makedirs(file_path_input)

        #Establir ruta on guardar la imatge comprimida
        file_path_output = os.path.join(script_dir, "compressed_image") 

        # Crear el directori en cas de no existir
        if not os.path.exists(file_path_output):
            os.makedirs(file_path_output)

        # Imatge CV2 rebuda al topic original_image
        cv_image = self.bridge.imgmsg_to_cv2(img_data, "bgr8")


        # Compresió mitjançant algoritme SPIHT
        if self.comp_type == "SPIHT":
            rospy.loginfo("Imatge rebuda, iniciant compresió amb SPIHT!") 

            # Assignar nom i format a la imatge original
            if self.historic == True:
                # Guardar historic de imatges originals
                img_number = int(len(os.listdir(file_path_input)))
                file_name = "img{}.pgm".format(str(img_number))
            else:
                # Sobreescriure imatge original
                file_name = "img.pgm"
            file_path_input = os.path.join(file_path_input, file_name)


            # Assignar nom i format a la imatge comprimida
            if self.historic == True:
                # Guardar historic de imatges comprimides
                img_number = int(len(os.listdir(file_path_output)))
                file_name = "img{}.ims".format(str(img_number))
            else:
                # Sobreescriure imatge comprimida
                file_name = "img.ims"
            file_path_output = os.path.join(file_path_output, file_name)


            # Passar imatge CV2 original a escala de grisos
            cv_image = cv.cvtColor(cv_image, cv.COLOR_RGB2GRAY)
            # Passar imatge CV2 en escala de grisos a PIL
            pil_image = Image.fromarray(cv_image)            
            # Guardar imatge PIL en escala de grisos
            pil_image.save(file_path_input)


            #Comprimir imagen en formato .ims
            run_imshrinker(file_path_input, file_path_output, script_dir)
            rospy.loginfo("Imatge comprimida correctament!")

            if os.path.exists(file_path_output):
        
                # Leer archivo por partes y publicarlo
                with open(file_path_output, 'rb') as imsfile:
                    bsplit_msg = BinarySplit()
                    chunk_size = 128
                    bsplit_msg.chunk_size = chunk_size
                    chunk = imsfile.read(chunk_size)
                    while chunk:
                        bsplit_msg.chunk_number += 1
                        bsplit_msg.data.append(chunk)
                        chunk = imsfile.read(chunk_size)

                    self.img_pub.publish(bsplit_msg)
                    rospy.loginfo("Imatge comprimida publicada!") 
            else: 
                rospy.logerr("No existeix el directori {}".format(file_path_output))

           

        # Compresió mitjançant algoritme DEBT
        elif self.comp_type == "DEBT":

            # Establir nom i format a la imatge comprimida
            rospy.loginfo("Imatge rebuda, iniciant compresió amb DEBT!")

            # Assignar nom i format a la imatge original
            if self.historic == True:
                # Guardar historic de imatges originals
                img_number = int(len(os.listdir(file_path_input)))
                file_name = "img{}.pgm".format(str(img_number))
            else:
                # Sobreescriure imatge original
                file_name = "img.pgm"
            file_path_input = os.path.join(file_path_input, file_name)


            # Assignar nom i format a la imatge comprimida
            if self.historic == True:
                # Guardar historic de imatges comprimides
                img_number = int(len(os.listdir(file_path_output)))
                file_name = "img{}.dbt".format(str(img_number))
            else:
                # Sobreescriure imatge comprimida
                file_name = "img.dbt"
            file_path_output = os.path.join(file_path_output, file_name)   


            # Passar imatge CV2 original a escala de grisos
            cv_image = cv.cvtColor(cv_image, cv.COLOR_RGB2GRAY)
            # Passar imatge CV2 en escala de grisos a PIL
            pil_image = Image.fromarray(cv_image)
            # Guardar imatge PIL en escala de grisos
            pil_image.save(file_path_input)

            # Comprimir imagen en formato .dbt
            run_debter(file_path_input, file_path_output, script_dir)
            rospy.loginfo("Imatge comprimida correctament!")

            if os.path.exists(file_path_output):
                 
                # Leer archivo por partes y publicarlo
                with open(file_path_output, 'rb') as dbtfile:
                    bsplit_msg = BinarySplit()
                    chunk_size = 128
                    bsplit_msg.chunk_size = chunk_size
                    chunk = dbtfile.read(chunk_size)
                    while chunk:
                        bsplit_msg.chunk_number += 1
                        bsplit_msg.data.append(chunk)
                        chunk = dbtfile.read(chunk_size)

                    self.img_pub.publish(bsplit_msg)
                    rospy.loginfo("Imatge comprimida publicada!")
                    
            else:
                rospy.logerr("No existeix el directori {}".format(file_path_output))


        # Compresió mitjançant algoritme JPEG2000
        elif self.comp_type == "JPEG2000":
            rospy.loginfo("Imatge rebuda, iniciant compresió amb JPEG2000!")

            # Assignar nom i format a la imatge original
            if self.historic == True:
                # Guardar historic de imatges originals
                img_number = int(len(os.listdir(file_path_input)))
                file_name = "img{}.jpg".format(str(img_number))
            else:
                # Sobreescriure imatge original
                file_name = "img.jpg"
            file_path_input = os.path.join(file_path_input, file_name)


            # Assignar nom i format a la imatge comprimida
            if self.historic == True:
                # Guardar historic de imatges comprimides
                img_number = int(len(os.listdir(file_path_output)))
                file_name = "img{}.jp2".format(str(img_number))
            else:
                # Sobreescriure imatge comprimida
                file_name = "img.jp2"
            file_path_output = os.path.join(file_path_output, file_name)   


            # Comprovar escala de grisos de la imatge CV2 original
            if self.grayscale:
                # Passar imatge CV2 original a escala de grisos
                cv_image = cv.cvtColor(cv_image, cv.COLOR_RGB2GRAY)
            else:
                # Canviar esquema de color de la imatge CV2 original
                cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)


            # Comprovar reescalat de la imatge CV2
            if self.img_height != 0 or self.img_width != 0:
                new_height = (self.img_height if self.img_height != 0
                              else cv_image.shape[0])
                new_width  = (self.img_width if self.img_width != 0
                              else cv_image.shape[1])

                # Reescalar imatge CV2 
                cv_image = cv.resize(cv_image, (new_width, new_height),
                                     interpolation=cv.INTER_AREA)

            # Passar imatge CV2 a PIL
            pil_image = Image.fromarray(cv_image)
            # Guardar imatge PIL
            pil_image.save(file_path_input)

            # Comprimir imagen en formato .jp2
            glymur.Jp2k(file_path_output, data=cv_image, cratios=[self.comp_ratio])
            #self.save_jpeg2000_img(pil_image, file_path_output, self.com_ratio)
        
            rospy.loginfo("Imatge comprimida correctament!")
            
            if os.path.exists(file_path_output):

                # Leer archivo por partes y publicarlo
                with open(file_path_output, 'rb') as jp2file:
                    bsplit_msg = BinarySplit()
                    
                    chunk_size = 128
                    bsplit_msg.chunk_size = chunk_size
                    chunk = jp2file.read(chunk_size)
                    while chunk:
                        bsplit_msg.chunk_number += 1
                        bsplit_msg.data.append(chunk)
                        chunk = jp2file.read(chunk_size)

                    self.img_pub.publish(bsplit_msg)
                    rospy.loginfo("Imatge comprimida publicada!")

            else:
                rospy.logerr("No existeix el directori {}".format(file_path_output))

        else: 
            rospy.logerr("Format de compressió no suportat")

    # def save_jpeg2000_img(self, img, img_path, ratio):
    #     img.save(file_path_output, quality_mode="rates", quality_layers=[ratio])

def main():
    rospy.init_node("compressor", anonymous=True, disable_signals=True)
    jpcompressor = ImageCompressor()

    print("[{:.0f}] Image compressor node started".format(time.time()))

    try:
        rospy.spin()
    except KeyboardInterrupt as e:
        print("[{:.0f}] Detected KeyboardInterrupt".format(time.time()))
    finally:
        rospy.signal_shutdown("KeyboardInterrupt detected")

    print("[{:.0f}] Image compressor node ended".format(time.time()))


if __name__ == "__main__":
    main()