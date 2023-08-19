#!/usr/bin/env python
# -*- coding: utf-8 -*-

from time import time
import rospy
import os
from sensor_msgs.msg import Image as ImageMsg
from img_compressor.msg import BinarySplit
import subprocess
import os

'''def run_imshrinker(input_file, output_file):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_file = os.path.join(script_dir, input_file)
    output_file = os.path.join(script_dir, output_file)

    # Construye el comando como una lista de strings 
    command = [os.path.join(script_dir, "imshrinker"), "d", input_file, output_file]
    
    # Ejecuta el comando. Ejemplo: ./imshrinker  d  ../test-data/Cnew425ppm.ims ../test-data/Dnew425ppm.ppm
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()

    stdout, stderr = process.communicate()
    print(stderr)

    if process.returncode != 0:
        rospy.logerr("imshrinker failed with error code: {}".format(process.returncode))
        rospy.logerr("stdout: {}, stderr: {}".format(stdout, stderr))'''

def run_imshrinker(input_file, output_file):
    script_dir = os.path.dirname(os.path.realpath(__file__))
    input_file = os.path.join(script_dir, input_file)
    output_file = os.path.join(script_dir, output_file)

    command = [os.path.join(script_dir, "imshrinker"), "d", input_file, output_file]
    try:
        stdout = subprocess.check_output(command)
        print(stdout)
    except subprocess.CalledProcessError as e:
        print("Error executing command: ", e.output)


'''def run_debter(input_file, output_file):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Construye el comando como una lista de strings
    command = [os.path.join(script_dir, "debter"), "-d", input_file, output_file]
    
    # Ejecuta el comando. Ejemplo: ./debter -d < lena.dbt > dlena.pgm
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()

    if process.returncode != 0:
        rospy.logerr("debter failed with error code: {}".format(process.returncode))
        rospy.logerr("stdout: {}, stderr: {}".format(stdout, stderr))'''

def run_debter(input_file, output_file):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_file = os.path.join(script_dir, input_file)
    output_file = os.path.join(script_dir, output_file)

    # Construye el comando como una única cadena
    command = "{} -d < {} > {}".format(os.path.join(script_dir, "debter"), input_file, output_file)
    
    # Ejecuta el comando. Ejemplo: ./debter -d < lena.dbt > dlena.pgm
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()

    if process.returncode != 0:
        rospy.logerr("debter failed with error code: {}".format(process.returncode))
        rospy.logerr("stdout: {}, stderr: {}".format(stdout, stderr))


class Joiner:
    def __init__(self):

        self.comp_type = "JPEG2000"
        img_compressor_type = "/img_compressor/type"
        if rospy.has_param(img_compressor_type):
            self.comp_type = rospy.get_param(img_compressor_type)

        self.img_sub = rospy.Subscriber("compressed_image", BinarySplit, self._img_callback)

        self.img_pub = rospy.Publisher("out", ImageMsg, queue_size = 1)
        if rospy.has_param('~outdir'):
            self.outdir = rospy.get_param('~outdir')
            if not "/" in self.outdir[-1]:
                self.outdir += "/"
            if not os.path.exists(self.outdir):
                os.makedirs(self.outdir)
        else:
            self.outdir = "/tmp/compressed_img/"
            os.makedirs(self.outdir)


    def _img_callback(self, msg):

        img_number = int(len(os.listdir(self.outdir)))
        
        # Comprovar el format de compresió
        if self.comp_type ==  "JPEG2000":
            rospy.loginfo("Imatge rebuda, iniciant decompresió amb JPEG2000!")
            str_format = ".jp2"
            # Join the image      
            with open(self.outdir + "img_" + str(img_number) + str_format, 'wb') as jp2file:
                for b_data in msg.data:
                    jp2file.write(b_data)

            rospy.loginfo("Imatge descomprimida correctament!")
                
        elif self.comp_type == "DEBT":
            rospy.loginfo("Imatge rebuda, iniciant decompresió amb DEBT!")
            str_format = ".dbt"
            # Join the image      
            with open(self.outdir + "img_" + str(img_number) + str_format, 'wb') as dbtfile:
                for b_data in msg.data:
                    dbtfile.write(b_data)

            #Executar descompresió amb DEBT
            run_debter("./compressed_image/lena.dbt", "./decompressed_image/lena.pgm")
            rospy.loginfo("Imatge descomprimida correctament!")
            
        elif self.comp_type == "SPIHT":
            rospy.loginfo("Imatge rebuda, iniciant decompresió amb SPIHT!")
            str_format = ".ims"
            # Join the image      
            with open(self.outdir + "img_" + str(img_number) + str_format, 'wb') as imsfile:
                for b_data in msg.data:
                    imsfile.write(b_data)
            
            #Executar descompresió amb SPIHT
            run_imshrinker("./compressed_image/lena.ims", "./decompressed_image/lena.pgm")
            rospy.loginfo("Imatge descomprimida correctament!")


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