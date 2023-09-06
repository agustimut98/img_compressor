#!/usr/bin/env python
# -*- coding: utf-8 -*-

from time import time
import rospy
import os
from sensor_msgs.msg import Image as ImageMsg
from img_compressor.msg import BinarySplit
import subprocess
import glymur
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

        # Establir ruta on guardar la imatge rebuda al topic compressed_image (../compressed_image)
        file_path_input = os.path.join(script_dir, "compressed_image")

        # Crear el directori en cas de no existir
        if not os.path.exists(file_path_input):
            os.makedirs(file_path_input)

        # Establir ruta on guardar la imatge descomprimida (../decompressed_image)
        file_path_output = os.path.join(script_dir, "decompressed_image")

        # Crear el directori en cas de no existir
        if not os.path.exists(file_path_output):
            os.makedirs(file_path_output)

        
        # Comprovar el format de compresió
        if self.comp_type ==  "JPEG2000":
            rospy.loginfo("Imatge rebuda, iniciant decompresió amb JPEG2000!")

            #Establir ruta on guardar la imatge comprimida segons el tipus d'algoritme (../compressed_image/JPEG2000)
            file_path_input_JPEG2000 = os.path.join(file_path_input, "JPEG2000")

            # Crear el directori en cas de no existir
            if not os.path.exists(file_path_input_JPEG2000):
                os.makedirs(file_path_input_JPEG2000)
            
            # Assignar nom i format a la imatge comprimida segons el paràmetre historic
            if self.historic == True:
                # Guardar historic de imatges comprimides (../compressed_image/JPEG2000/imgX.jp2)
                img_number = int(len(os.listdir(file_path_input_JPEG2000)))
                file_name = "img{}.jp2".format(str(img_number))
            else:
                # Sobreescriure imatge comprmida (../compressed_image/JPEG2000/img.jp2)
                file_name = "img.jp2"
            file_path_input_JPEG2000 = os.path.join(file_path_input_JPEG2000, file_name)


            #Establir ruta on guardar la imatge descomprimida segons el tipus d'algoritme (../decompressed_image/JPEG2000)
            file_path_output_JPEG2000 = os.path.join(file_path_output, "JPEG2000")

            # Crear el directori en cas de no existir
            if not os.path.exists(file_path_output_JPEG2000):
                os.makedirs(file_path_output_JPEG2000)

            # Assignar nom i format a la imatge descomprmida segons el paràmetre historic
            if self.historic == True:
                # Guardar historic de imatges descomprimides (../decompressed_image/JPEG2000/imgX.jpg)
                img_number = int(len(os.listdir(file_path_output_JPEG2000)))
                file_name = "img{}.jpg".format(str(img_number))
            else:
                # Sobreescriure imatge descomprmida (../decompressed_image/JPEG2000/img.jpg)
                file_name = "img.jpg"
            file_path_output_JPEG2000 = os.path.join(file_path_output_JPEG2000, file_name)


            # Join the image      
            with open(file_path_input_JPEG2000, 'wb') as jp2file:
                for b_data in msg.data:
                    jp2file.write(b_data)

            # Executar descompresió amb JPEG2000
            Image.fromarray(glymur.Jp2k(file_path_input_JPEG2000)[:]).save(file_path_output_JPEG2000)
            rospy.loginfo("Imatge descomprimida correctament!")

            # Publicar imatge al topic decompressed_image
            if self.grayscale == True:
                img = cv2.imread(file_path_output_JPEG2000, cv2.IMREAD_GRAYSCALE)  # Leer como imagen en escala de grises
                img_msg = self.bridge.cv2_to_imgmsg(img, "mono8")  # Convertir a mensaje de ROS como imagen de 8 bits en monocromo
            else:
                img = cv2.imread(file_path_output_JPEG2000, cv2.IMREAD_COLOR)  # Leer como imagen en color
                img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")  # Convertir a mensaje de ROS como imagen de 8 bits en color
            
            self.img_pub.publish(img_msg)
            rospy.loginfo("Imatge publicada!")
            
                
        elif self.comp_type == "DEBT":
            rospy.loginfo("Imatge rebuda, iniciant decompresió amb DEBT!")

            #Establir ruta on guardar la imatge comprimida segons el tipus d'algoritme (../compressed_image/DEBT)
            file_path_input_DEBT = os.path.join(file_path_input, "DEBT")

            # Crear el directori en cas de no existir
            if not os.path.exists(file_path_input_DEBT):
                os.makedirs(file_path_input_DEBT)

            # Assignar nom i format a la imatge comprimida segons el paràmetre historic
            if self.historic == True:
                # Guardar historic de imatges comprimides (../compressed_image/JPEG2000/imgX.dbt)
                img_number = int(len(os.listdir(file_path_input_DEBT)))
                file_name = "img{}.dbt".format(str(img_number))
            else:
                # Sobreescriure imatge comprmida (../compressed_image/JPEG2000/img.dbt)
                file_name = "img.dbt"
            file_path_input_DEBT = os.path.join(file_path_input_DEBT, file_name)


            #Establir ruta on guardar la imatge descomprimida segons el tipus d'algoritme (../decompressed_image/JPEG2000)
            file_path_output_DEBT = os.path.join(file_path_output, "DEBT")

            # Crear el directori en cas de no existir
            if not os.path.exists(file_path_output_DEBT):
                os.makedirs(file_path_output_DEBT)

            # Assignar nom i format a la imatge descomprimida segons el paràmetre historic
            if self.historic == True:
                # Guardar historic de imatges descomprimides (../decompressed_image/JPEG2000/imgX.pgm)           
                img_number = int(len(os.listdir(file_path_output_DEBT)))
                file_name = "img{}.pgm".format(str(img_number))
            else:
                # Sobreescriure imatge descomprmida (../decompressed_image/JPEG2000/img.pgm)            
                file_name = "img.pgm"
            file_path_output_DEBT = os.path.join(file_path_output_DEBT, file_name)


            # Join the image      
            with open(file_path_input_DEBT, 'wb') as dbtfile:
                for b_data in msg.data:
                    dbtfile.write(b_data)

            #Executar descompresió amb DEBT
            run_debter(file_path_input_DEBT, file_path_output_DEBT, script_dir)
            rospy.loginfo("Imatge descomprimida correctament!")

            # Publicar imatge al topic decompressed_image
            img = cv2.imread(file_path_output_DEBT, cv2.IMREAD_GRAYSCALE)  # Leer como imagen en escala de grises
            img_msg = self.bridge.cv2_to_imgmsg(img, "mono8")  # Convertir a mensaje de ROS como imagen de 8 bits en monocromo
            self.img_pub.publish(img_msg)
            rospy.loginfo("Imatge descomprimida publicada!")

            
        elif self.comp_type == "SPIHT":
            rospy.loginfo("Imatge rebuda, iniciant decompresió amb SPIHT!")

            #Establir ruta on guardar la imatge comprimida segons el tipus d'algoritme (../compressed_image/SPIHT)
            file_path_input_SPIHT = os.path.join(file_path_input, "SPIHT")

            # Crear el directori en cas de no existir
            if not os.path.exists(file_path_input_SPIHT):
                os.makedirs(file_path_input_SPIHT)

            # Assignar nom i format a la imatge comprimida segons el paràmetre historic
            if self.historic == True:
                # Guardar historic de imatges comprimides (../compressed_image/SPIHT/imgX.ims)
                img_number = int(len(os.listdir(file_path_input_SPIHT)))
                file_name = "img{}.ims".format(str(img_number))
            else:
                # Sobreescriure imatge comprmida (../compressed_image/SPIHT/img.ims)
                file_name = "img.ims"
            file_path_input_SPIHT = os.path.join(file_path_input_SPIHT, file_name)


            #Establir ruta on guardar la imatge descomprimida segons el tipus d'algoritme (../decompressed_image/SPIHT)
            file_path_output_SPIHT = os.path.join(file_path_output, "SPIHT")

            # Crear el directori en cas de no existir
            if not os.path.exists(file_path_output_SPIHT):
                os.makedirs(file_path_output_SPIHT)

            # Assignar nom i format a la imatge descomprimida segons els paràmetres graycale i historic
            if self.grayscale == True:
                if self.historic == True:
                    # Guardar historic de imatges descomprimides  (../decompressed_image/SPIHT/imgX.pgm)              
                    img_number = int(len(os.listdir(file_path_output_SPIHT)))
                    file_name = "img{}.pgm".format(str(img_number))
                else:
                    # Sobreescriure imatge descomprmida (../decompressed_image/SPIHT/img.pgm)       
                    file_name = "img.pgm"
                file_path_output_SPIHT = os.path.join(file_path_output_SPIHT, file_name)

            else:
                if self.historic == True:
                    # Guardar historic de imatges descomprimides  (../decompressed_image/SPIHT/imgX.ppm)              
                    img_number = int(len(os.listdir(file_path_output_SPIHT)))
                    file_name = "img{}.ppm".format(str(img_number))
                else:
                    # Sobreescriure imatge descomprmida (../decompressed_image/SPIHT/img.ppm)       
                    file_name = "img.ppm"
                file_path_output_SPIHT = os.path.join(file_path_output_SPIHT, file_name)


            # Join the image      
            with open(file_path_input_SPIHT, 'wb') as imsfile:
                for b_data in msg.data:
                    imsfile.write(b_data)

            #Executar descompresió amb SPIHT
            run_imshrinker(file_path_input_SPIHT, file_path_output_SPIHT, script_dir)
            rospy.loginfo("Imatge descomprimida correctament!")

            # Publicar imatge al topic decompressed_image
            img = cv2.imread(file_path_output_SPIHT, cv2.IMREAD_GRAYSCALE)  # Leer como imagen en escala de grises
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