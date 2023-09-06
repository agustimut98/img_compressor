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

        #Establir ruta on guardar la imatge rebuda al topic original_image (../original_image)
        file_path_input = os.path.join(script_dir, "original_image")

        # Crear el directori en cas de no existir
        if not os.path.exists(file_path_input):
            os.makedirs(file_path_input)

        #Establir ruta on guardar la imatge comprimida (../compressed_image)
        file_path_output = os.path.join(script_dir, "compressed_image") 

        # Crear el directori en cas de no existir
        if not os.path.exists(file_path_output):
            os.makedirs(file_path_output)

        # Imatge CV2 rebuda al topic original_image
        cv_image = self.bridge.imgmsg_to_cv2(img_data, "bgr8")


        # Compresió mitjançant algoritme SPIHT
        if self.comp_type == "SPIHT":
            rospy.loginfo("Imatge rebuda, iniciant compresió amb SPIHT!")

            #Establir ruta on guardar la imatge rebuda al topic original_image segons el tipus d'algoritme (../original_image/SPIHT)
            file_path_input_SPIHT = os.path.join(file_path_input, "SPIHT")

            # Crear el directori en cas de no existir
            if not os.path.exists(file_path_input_SPIHT):
                os.makedirs(file_path_input_SPIHT)

            # Assignar nom i format a la imatge original segons el paràmetres grayscale i historic
            if self.grayscale == True:
                if self.historic == True:
                    # Guardar historic de imatges originals (../original_image/SPIHT/imgX.pgm)
                    img_number = int(len(os.listdir(file_path_input_SPIHT)))
                    file_name = "img{}.pgm".format(str(img_number))
                else:
                    # Sobreescriure imatge original (../original_image/SPIHT/img.pgm)
                    file_name = "img.pgm"
                file_path_input_SPIHT = os.path.join(file_path_input_SPIHT, file_name)

            else:
                if self.historic == True:
                    # Guardar historic de imatges originals (../original_image/SPIHT/imgX.ppm)
                    img_number = int(len(os.listdir(file_path_input_SPIHT)))
                    file_name = "img{}.ppm".format(str(img_number))
                else:
                    # Sobreescriure imatge original (../original_image/SPIHT/img.ppm)
                    file_name = "img.ppm"
                file_path_input_SPIHT = os.path.join(file_path_input_SPIHT, file_name)


            #Establir ruta on guardar la imatge comprimida segons el tipus d'algoritme (../compressed_image/SPIHT)
            file_path_output_SPIHT = os.path.join(file_path_output, "SPIHT")

            # Crear el directori en cas de no existir
            if not os.path.exists(file_path_output_SPIHT):
                os.makedirs(file_path_output_SPIHT)

            # Assignar nom i format a la imatge comprimida segons el parametre historic
            if self.historic == True:
                # Guardar historic de imatges comprimides (../compressed_image/SPIHT/imgX.ims)
                img_number = int(len(os.listdir(file_path_output_SPIHT)))
                file_name = "img{}.ims".format(str(img_number))
            else:
                # Sobreescriure imatge comprimida (../compressed_image/SPIHT/img.ims)
                file_name = "img.ims"
            file_path_output_SPIHT = os.path.join(file_path_output_SPIHT, file_name)

            if self.grayscale == True:
                # Passar imatge CV2 original a escala de grisos
                cv_image = cv.cvtColor(cv_image, cv.COLOR_RGB2GRAY)
                # Passar imatge CV2 en escala de grisos a PIL
                pil_image = Image.fromarray(cv_image)            
                # Guardar imatge PIL en escala de grisos (.pgm)
                pil_image.save(file_path_input_SPIHT)
            else: 
                # Guardar imatge en color (.ppm)
                cv.imwrite(file_path_input_SPIHT, cv_image)


            #Comprimir imagen en formato .ims
            run_imshrinker(file_path_input_SPIHT, file_path_output_SPIHT, script_dir)
            rospy.loginfo("Imatge comprimida correctament!")

            if os.path.exists(file_path_output_SPIHT):
        
                # Leer archivo por partes y publicarlo
                with open(file_path_output_SPIHT, 'rb') as imsfile:
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
                rospy.logerr("No existeix el directori {}".format(file_path_output_SPIHT))

           

        # Compresió mitjançant algoritme DEBT
        elif self.comp_type == "DEBT":
            rospy.loginfo("Imatge rebuda, iniciant compresió amb DEBT!")

            #Establir ruta on guardar la imatge rebuda al topic original_image segons el tipus d'algoritme (../original_image/DEBT)
            file_path_input_DEBT = os.path.join(file_path_input, "DEBT")

            # Crear el directori en cas de no existir
            if not os.path.exists(file_path_input_DEBT):
                os.makedirs(file_path_input_DEBT)

            # Assignar nom i format a la imatge original segons el paràmetre historic
            if self.historic == True:
                # Guardar historic de imatges originals (../original_image/DEBT/imgX.pgm)
                img_number = int(len(os.listdir(file_path_input_DEBT)))
                file_name = "img{}.pgm".format(str(img_number))
            else:
                # Sobreescriure imatge original (../original_image/DEBT/img.pgm)
                file_name = "img.pgm"
            file_path_input_DEBT = os.path.join(file_path_input_DEBT, file_name)


            #Establir ruta on guardar la imatge comprimida segons el tipus d'algoritme (../compressed_image/DEBT)
            file_path_output_DEBT = os.path.join(file_path_output, "DEBT")

            # Crear el directori en cas de no existir
            if not os.path.exists(file_path_output_DEBT):
                os.makedirs(file_path_output_DEBT)

            # Assignar nom i format a la imatge comprimida segons el paràmetre historic
            if self.historic == True:
                # Guardar historic de imatges comprimides (../compressed_image/DEBT/imgX.dbt)
                img_number = int(len(os.listdir(file_path_output_DEBT)))
                file_name = "img{}.dbt".format(str(img_number))
            else:
                # Sobreescriure imatge comprimida (../compressed_image/DEBT/img.dbt)
                file_name = "img.dbt"
            file_path_output_DEBT = os.path.join(file_path_output_DEBT, file_name)  


            # Passar imatge CV2 original a escala de grisos
            cv_image = cv.cvtColor(cv_image, cv.COLOR_RGB2GRAY)
            # Passar imatge CV2 en escala de grisos a PIL
            pil_image = Image.fromarray(cv_image)
            # Guardar imatge PIL en escala de grisos
            pil_image.save(file_path_input_DEBT)

            # Comprimir imagen en formato .dbt
            run_debter(file_path_input_DEBT, file_path_output_DEBT, script_dir)
            rospy.loginfo("Imatge comprimida correctament!")

            if os.path.exists(file_path_output_DEBT):
                 
                # Leer archivo por partes y publicarlo
                with open(file_path_output_DEBT, 'rb') as dbtfile:
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
                rospy.logerr("No existeix el directori {}".format(file_path_output_DEBT))


        # Compresió mitjançant algoritme JPEG2000
        elif self.comp_type == "JPEG2000":
            rospy.loginfo("Imatge rebuda, iniciant compresió amb JPEG2000!")

            #Establir ruta on guardar la imatge rebuda al topic original_image segons el tipus d'algoritme (../original_image/JPEG2000)
            file_path_input_JPEG2000 = os.path.join(file_path_input, "JPEG2000")

            # Crear el directori en cas de no existir
            if not os.path.exists(file_path_input_JPEG2000):
                os.makedirs(file_path_input_JPEG2000)

            # Assignar nom i format a la imatge original segons el paràmetre històric
            if self.historic == True:
                # Guardar historic de imatges originals (../original_image/JPEG2000/imgX.jpg)
                img_number = int(len(os.listdir(file_path_input_JPEG2000)))
                file_name = "img{}.jpg".format(str(img_number))
            else:
                # Sobreescriure imatge original (../original_image/JPEG2000/img.jpg)
                file_name = "img.jpg"
            file_path_input_JPEG2000 = os.path.join(file_path_input_JPEG2000, file_name)


            #Establir ruta on guardar la imatge comprimida segons el tipus d'algoritme (../compressed_image/JPEG2000)
            file_path_output_JPEG2000 = os.path.join(file_path_output, "JPEG2000") 

            # Crear el directori en cas de no existir
            if not os.path.exists(file_path_output_JPEG2000):
                os.makedirs(file_path_output_JPEG2000)

            # Assignar nom i format a la imatge comprimida segons el paràmetre historic
            if self.historic == True:
                # Guardar historic de imatges comprimides (../compressed_image/JPEG2000/imgX.jpg)
                img_number = int(len(os.listdir(file_path_output_JPEG2000)))
                file_name = "img{}.jp2".format(str(img_number))
            else:
                # Sobreescriure imatge comprimida (../compressed_image/JPEG2000/img.jpg)
                file_name = "img.jp2"
            file_path_output_JPEG2000 = os.path.join(file_path_output_JPEG2000, file_name)   


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
            pil_image.save(file_path_input_JPEG2000)

            # Comprimir imagen en formato .jp2
            glymur.Jp2k(file_path_output_JPEG2000, data=cv_image, cratios=[self.comp_ratio])
            #self.save_jpeg2000_img(pil_image, file_path_output_JPEG2000, self.comp_ratio)
        
            rospy.loginfo("Imatge comprimida correctament!")
            
            if os.path.exists(file_path_output_JPEG2000):

                # Leer archivo por partes y publicarlo
                with open(file_path_output_JPEG2000, 'rb') as jp2file:
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
                rospy.logerr("No existeix el directori {}".format(file_path_output_JPEG2000))

        else: 
            rospy.logerr("Format de compressió no suportat")

    # def save_jpeg2000_img(self, img, img_path, ratio):
    #     img.save(img_path, quality_mode="rates", quality_layers=[ratio])

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