# cirtesu_common_python_img_compressor


Hay que instalar opencv para python2
Hay que instalar numpy
Antes de instalar pillow hay que instalar las librerias libjpeg-dev y libopenjp2-7-dev
Luego hay que instalar pillow
Hay que instalar el paquete ros-melodic-cv-bridge
Si quieres probarlo con la camara hay que instalar el paquete ros-melodic-cv-camera

Recuerda hacer un catkin_make para que el msg custom se compile y despues vovler a hacer el source del catkin_ws

Un ejemplo de ejecucion sería el siguiente
rosrun cirtesu_common_python_img_compressor img_compressor.py /camera:=/cv_camera/image_raw _/img_compressor/grayscale:=False _/img_compressor/width:=320 _/img_compressor/height:=240

El que junta la imagen no esta terminado, la imagen esta puesta ya para unirse y crear el binario, falta transformarlo en un nuevo Image y meterlo en la cola que toque.
