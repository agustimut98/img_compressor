#!/usr/bin/env python
# -*- coding: utf-8 -*-

from time import time
import rospy
import cv2 as cv
from sensor_msgs.msg import Image as ImageMsg
from img_compressor.msg import BinarySplit
#from mine_detection.msg import det
#from mine_detection.msg import mine_detections
from cv_bridge import CvBridge
from PIL import Image
import subprocess
import os
import time


def run_imshrinker(input_file, output_file):
    # Comprobar rutas
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_file = os.path.join(script_dir, input_file)
    output_file = os.path.join(script_dir, output_file)

    # Construye el comando como una lista de strings 
    command = [os.path.join(script_dir, "imshrinker"), "c0.9" , input_file, output_file]
    
    # Ejecuta el comando. Ejemplo: ./imshrinker  c0.7 ../test-data/new425.ppm ../test-data/Cnew425ppm.ims
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()

    if process.returncode != 0:
        rospy.logerr("imshrinker failed with error code: {}".format(process.returncode))
        rospy.logerr("stdout: {}, stderr: {}".format(stdout, stderr))

'''def run_debter(input_file, output_file):
    # Comprobar rutas
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_file = os.path.join(script_dir, input_file)
    output_file = os.path.join(script_dir, output_file)

    # Construye el comando como una lista de strings
    command = [os.path.join(script_dir, "debter"), "-t", "cdf-9/7", "-r", "2", "-q", "4", "-f", "1", "<", input_file, ">", output_file, "2", "/dev/null"]
    
    # Ejecuta el comando. Ejemplo: ./debter -t cdf-9/7 -r 2 -q 4 -f 1 < lena.pgm > lena.dbt 2> log.txt
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()

    if process.returncode != 0:
        rospy.logerr("debter failed with error code: {}".format(process.returncode))
        rospy.logerr("stdout: {}, stderr: {}".format(stdout, stderr))'''

def run_debter(input_file, output_file):
    # Comprobar rutas
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_file = os.path.join(script_dir, input_file)
    output_file = os.path.join(script_dir, output_file)

    # Construye el comando como una lista de strings
    command = [os.path.join(script_dir, "debter"), "-t", "cdf-9/7", "-r", "2", "-q", "4", "-f", "1"]

    # Abre los archivos de entrada y salida
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile, open(os.devnull, 'w') as DEVNULL:

        # Ejecuta el comando
        process = subprocess.Popen(command, stdin=infile, stdout=outfile, stderr=DEVNULL)
        process.communicate()

    '''if process.returncode != 0:
        rospy.logerr("debter failed with error code: {}".format(process.returncode))
        # Aquí no se podrá obtener stderr porque se redirigió a /dev/null'''

class ImageCompressor:
    def __init__(self):

        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("original_image", ImageMsg, self.img_callback)
        self.img_pub = rospy.Publisher("compressed_image", BinarySplit, queue_size = 10)

        self.comp_type = "JPEG2000"
        img_compressor_type = "/img_compressor/type"
        if rospy.has_param(img_compressor_type):
            self.comp_type = rospy.get_param(img_compressor_type) 

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

        # mine_img=img_data.image_rect_color
        cv_image = self.bridge.imgmsg_to_cv2(img_data, "bgr8")

        if self.comp_type == "SPIHT":
            rospy.loginfo("Imatge rebuda, iniciant compresió amb SPIHT!")
            script_dir = os.path.dirname(os.path.abspath(__file__))
            file_path_input = os.path.join(script_dir, "raw_image/img.pgm")   #Establir ruta on guardar la imatge rebuda al topic original_image
            file_path_output = os.path.join(script_dir, "compressed_image/img.ims") #Establir ruta on guardar la imatge comprimida

            cv_image = cv.cvtColor(cv_image, cv.COLOR_RGB2GRAY)
            pil_image = Image.fromarray(cv_image)
            pil_image.save(file_path)

            #Comprimir imagen en formato .ims
            run_imshrinker("./raw_image/lena.pgm", "./compressed_image/lena.ims")
            rospy.loginfo("Imatge comprimida correctament!")

            file_path = os.path.join(script_dir, "compressed_image/lena.ims")
            

            max_attempts = 10
            attempts = 0
            while(attempts < max_attempts):

                if os.path.exists(file_path):
                    try: 
                        # Leer archivo por partes y publicarlo
                        with open(file_path, 'rb') as imsfile:
                            bsplit_msg = BinarySplit()
                            chunk_size = 128
                            bsplit_msg.chunk_size = chunk_size
                            chunk = imsfile.read(chunk_size)
                            while chunk:
                                bsplit_msg.chunk_number += 1
                                bsplit_msg.data.append(chunk)
                                # Tal vegda sigui bsplit_msg.data.extend(chunk)
                                chunk = imsfile.read(chunk_size)

                            self.img_pub.publish(bsplit_msg)
                            rospy.loginfo("Imatge comprimida publicada!")
                            break

                    except IOError:
                        # Si obtenemos un error de E/S al intentar abrir el archivo, asumimos que todavia no esta listo
                        pass
                # Si el archivo no existe o no se pudo abrir, esperamos un segundo y lo intentamos de nuevo
                time.sleep(1)
                attempts += 1

            if attempts == max_attempts:
                rospy.logerr("No se pudo abrir el archivo despues de {} intentos".format(max_attempts))
           

        if self.comp_type == "DEBT":
            rospy.loginfo("Imatge rebuda, iniciant compresió amb DEBT!")
            
            '''#cv_image = cv.cvtColor(cv_image, cv.COLOR_RGB2GRAY)
            # no se quina de les dues exactament
            cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
            pil_image = Image.fromarray(cv_image)
            pil_image.save("/tmp/img.pgm")'''

            # Comprimir imagen en formato .dbt
            run_debter("./raw_image/lena.pgm", "./compressed_image/lena.dbt")
            rospy.loginfo("Imatge comprimida correctament!")

            script_dir = os.path.dirname(os.path.abspath(__file__))
            file_path = os.path.join(script_dir, "compressed_image/lena.dbt")

            max_attempts = 10
            attempts = 0
            while(attempts < max_attempts):

                if os.path.exists(file_path):
                    try: 
                        # Leer archivo por partes y publicarlo
                        with open(file_path, 'rb') as dbtfile:
                            bsplit_msg = BinarySplit()
                            chunk_size = 128
                            bsplit_msg.chunk_size = chunk_size
                            chunk = dbtfile.read(chunk_size)
                            while chunk:
                                bsplit_msg.chunk_number += 1
                                bsplit_msg.data.append(chunk)
                                # Tal vegda sigui bsplit_msg.data.extend(chunk)
                                chunk = dbtfile.read(chunk_size)

                            self.img_pub.publish(bsplit_msg)
                            rospy.loginfo("Imatge comprimida publicada!")
                            break

                    except IOError:
                        # Si obtenemos un error de E/S al intentar abrir el archivo, asumimos que todavia no esta listo
                        pass
                # Si el archivo no existe o no se pudo abrir, esperamos un segundo y lo intentamos de nuevo
                time.sleep(1)
                attempts += 1

            if attempts == max_attempts:
                rospy.logerr("No se pudo abrir el archivo despues de {} intentos".format(max_attempts))


        elif self.comp_type == "JPEG2000":
            rospy.loginfo("Imatge rebuda, iniciant compresió amb JPEG2000!")
            #Comprobar parametros de compresion
            if self.grayscale:
                cv_image = cv.cvtColor(cv_image, cv.COLOR_RGB2GRAY)
            else:
                cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)

            if self.img_height != 0 or self.img_width != 0:
                new_height = (self.img_height if self.img_height != 0
                              else cv_image.shape[0])
                new_width  = (self.img_width if self.img_width != 0
                              else cv_image.shape[1])

                cv_image = cv.resize(cv_image, (new_width, new_height),
                                     interpolation=cv.INTER_AREA)

            pil_image = Image.fromarray(cv_image)
            pil_image.save("/tmp/img.pgm")

            # Comprimir imagen en formato .jp2
            self.save_jpeg2000_img(pil_image, "/tmp", 500)
            
            # Leer archivo por partes y publicarlo
            with open("/tmp/img.jp2", 'rb') as jp2file:
                bsplit_msg = BinarySplit()
                
                chunk_size = 128
                bsplit_msg.chunk_size = chunk_size
                chunk = jp2file.read(chunk_size)
                while chunk:
                    bsplit_msg.chunk_number += 1
                    bsplit_msg.data.append(chunk)
                    # Tal vegda sigui bsplit_msg.data.extend(chunk)
                    chunk = jp2file.read(chunk_size)

                self.img_pub.publish(bsplit_msg)

    def save_jpeg2000_img(self, img, img_path, ratio):
        img.save("{}/img.jp2".format(img_path), quality_mode="rates",
                quality_layers=[ratio])


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