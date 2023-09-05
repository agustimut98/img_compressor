#!/usr/bin/env python
# -*- coding: utf-8 -*-

from time import time
import rospy
import os
from sensor_msgs.msg import Image as ImageMsg
from img_compressor_v2.msg import BinarySplit
import subprocess
# import glymur
from PIL import Image
from cv_bridge import CvBridge
import cv2


def run_imshrinker(input_file, output_file, script_dir):

    # Construye el comando como una lista de strings 
    command = [os.path.join(script_dir, "imshrinker"), "d", input_file, output_file]
    
    # Ejecuta el comando. Ejemplo de REDME.md: ./imshrinker  d  ../test-data/Cnew425ppm.ims ../test-data/Dnew425ppm.ppm
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()


def run_debter(input_file, output_file, script_dir):

    # Construye el comando como una lista de strings
    command = [os.path.join(script_dir, "debter"), "-d"]
    
    # Abre los archivos de entrada y salida
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile, open(os.devnull, 'w') as DEVNULL:

        # Ejecuta el comando. Ejemplo de help.txt: ./debter -d < lena.dbt > dlena.pgm
        process = subprocess.Popen(command, stdin=infile, stdout=outfile, stderr=DEVNULL)
        process.communicate()


class Joiner:
    def __init__(self):

        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("compressed_image", BinarySplit, self._img_callback)
        self.img_pub = rospy.Publisher("decompressed_image", ImageMsg, queue_size = 10)

        self.comp_type = "JPEG2000"
        img_compressor_type = "/img_compressor_v2/type"
        if rospy.has_param(img_compressor_type):
            self.comp_type = rospy.get_param(img_compressor_type)


        ''' Exemple de com establir rutes amb parametres del launch
        if rospy.has_param('~outdir'):
            self.outdir = rospy.get_param('~outdir')
            if not "/" in self.outdir[-1]:
                self.outdir += "/"
            if not os.path.exists(self.outdir):
                os.makedirs(self.outdir)
        else:
            self.outdir = "/tmp/compressed_img/"
            os.makedirs(self.outdir)
        '''


    def _img_callback(self, msg):
        script_dir = os.path.dirname(os.path.abspath(__file__))

        # Establir ruta on guardar la imatge rebuda al topic compressed_image
        file_path_input = os.path.join(script_dir, "compressed_image")

        # Crear el directori en cas de no existir
        if not os.path.exists(file_path_input):
            os.makedirs(file_path_input)

        # Establir ruta on guardar la imatge descomprimida
        file_path_output = os.path.join(script_dir, "decompressed_image")

        # Crear el directori en cas de no existir
        if not os.path.exists(file_path_output):
            os.makedirs(file_path_output)

        # Comprovar nombre de fitxers en aquesta ruta, assignar nom i format a la imatge descomprimida
        img_number = int(len(os.listdir(file_path_output)))
        file_name = "img{}.pgm".format(str(img_number))
        file_path_output = os.path.join(file_path_output,file_name)
        
        # Comprovar el format de compresió
        if self.comp_type ==  "JPEG2000":
            rospy.loginfo("Imatge rebuda, iniciant decompresió amb JPEG2000!")
            
            # Establir nom i format a la imatge comprimida
            file_name = "img.jp2"
            file_path_input = os.path.join(file_path_input, file_name)

            # Join the image      
            with open(file_path_input, 'wb') as jp2file:
                for b_data in msg.data:
                    jp2file.write(b_data)

            img = cv2.imread(file_path_input, cv2.IMREAD_GRAYSCALE)
            img = cv2.imwrite(file_path_output)

            # Executar descompresió amb JPEG2000
            #Image.fromarray(glymur.Jp2k(file_path_input)[:]).save(file_path_output)
            #rospy.loginfo("Imatge descomprimida correctament!")

            # Publicar imatge al topic decompressed_image
            img = cv2.imread(file_path_input, cv2.IMREAD_GRAYSCALE)  # Leer como imagen en escala de grises
            img_msg = self.bridge.cv2_to_imgmsg(img, "mono8")  # Convertir a mensaje de ROS como imagen de 8 bits en monocromo
            self.img_pub.publish(img_msg)
            rospy.loginfo("Imatge descomprimida publicada!")
                

        elif self.comp_type == "DEBT":
            rospy.loginfo("Imatge rebuda, iniciant decompresió amb DEBT!")

            # Establir nom i format a la imatge comprimida
            file_name = "img.dbt"
            file_path_input = os.path.join(file_path_input,file_name)

            # Join the image      
            with open(file_path_input, 'wb') as dbtfile:
                for b_data in msg.data:
                    dbtfile.write(b_data)

            #Executar descompresió amb DEBT
            run_debter(file_path_input, file_path_output, script_dir)
            rospy.loginfo("Imatge descomprimida correctament!")

            # Publicar imatge al topic decompressed_image
            img = cv2.imread(file_path_output, cv2.IMREAD_GRAYSCALE)  # Leer como imagen en escala de grises
            img_msg = self.bridge.cv2_to_imgmsg(img, "mono8")  # Convertir a mensaje de ROS como imagen de 8 bits en monocromo
            self.img_pub.publish(img_msg)
            rospy.loginfo("Imatge descomprimida publicada!")

            
        elif self.comp_type == "SPIHT":
            rospy.loginfo("Imatge rebuda, iniciant decompresió amb SPIHT!")

            # Establir nom i format a la imatge comprimida
            file_name = "img.ims"
            file_path_input = os.path.join(file_path_input,file_name)

            # Join the image      
            with open(file_path_input, 'wb') as imsfile:
                for b_data in msg.data:
                    imsfile.write(b_data)

            #Executar descompresió amb SPIHT
            run_imshrinker(file_path_input, file_path_output, script_dir)
            rospy.loginfo("Imatge descomprimida correctament!")

            # Publicar imatge al topic decompressed_image
            img = cv2.imread(file_path_output, cv2.IMREAD_GRAYSCALE)  # Leer como imagen en escala de grises
            img_msg = self.bridge.cv2_to_imgmsg(img, "mono8")  # Convertir a mensaje de ROS como imagen de 8 bits en monocromo
            self.img_pub.publish(img_msg)
            rospy.loginfo("Imatge descomprimida publicada!")

        else: 
            rospy.logerr("Format de compressió no suportat")

def main():
    rospy.init_node("joiner", anonymous=True, disable_signals=True)
    jpjoiner = Joiner()
    print("[{:.0f}] Image joiner node started".format(time()))

    try:
        rospy.spin()
    except KeyboardInterrupt as e:
        print("[{:.0f}] Detected KeyboardInterrupt".format(time()))
    finally:
        rospy.signal_shutdown("KeyboardInterrupt detected")

    print("[{:.0f}] Image joiner node ended".format(time()))


if __name__ == "__main__":
    main()