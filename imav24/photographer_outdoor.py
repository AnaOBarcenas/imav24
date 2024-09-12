#!/usr/bin/env python3

import rclpy 
import math
import time
import numpy as np
import cv2 as cv
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleGlobalPosition
from std_msgs.msg import Empty, Float32, Bool, String
#from sensor_msgs.msg import Image

from cv_bridge import CvBridge 

from PIL import Image
import piexif


class PhotoOutdoor(Node):
    def __init__(self):
        super().__init__('photographer_outdoor')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        #Convertir imagenes entre ROS y OpenCV
        #Se crea la subscripción  que envia imagenes
        self.Bridge = CvBridge() 
        self.subscription = self.create_subscription(Image, '/oak/rgb/image_raw', self.listener_callback, 10)
        self.subscription =self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position',self.gps_callback,10)

        self.image_taken = False #Indicar si ya se tommo la foto

        self.latest_lat = None
        self.latest_lon = None
        self.latest_photo = None
        self.count_photo = 0 #Contador de fotos tomadas

        self.timers = self.create_timer(1.0,self.capture_photo)#Temporizador para que llame a la funcion

        self.image_taken = False
    
    def listener_callback(self, data):

        self.latest_photo = self.Bridge.imgmsg_to_cv2(data, "bgr8")

    def gps_callback(self,msg):
        self.latest_lat = msg.lat #latitud en grados
        self.latest_lon = msg.lon #longitud en grados
        self.get_logger().info(f"GPS recibido:lat{self.latest_lat},lon{self.latest_lon}")

    def to_deg(self,value,loc):
        if value < 0:
            loc_value = loc[0] #Usa S para latitud o W para longitud
        else:
            loc_value = loc[1] #Usa N para latitud o E para longitud
        
        #Conversión
        abs_value = abs(value)
        deg = int(abs_value)
        min = int((abs_value - deg) * 60)
        sec = int((abs_value - deg - min / 60)*3600000)

        return ((deg,1),(min,1),(sec,100)),loc_value

    def capture_photo(self):
        
        timestamp = int(time.time())
        filename = f"fotos_mapa/image_{self.count_photo}_{timestamp}.png"

        cv.imwrite(filename, self.latest_photo)  # Guardar la imagen
        image = Image.open(filename) 

        lat_deg,lat_ref = self.to_deg(self.latest_lat,["S","N"])
        lon_deg,lon_ref = self.to_deg(self.latest_lon,["W","E"])

        exif_dict = {
            "GPS": {
                piexif.GPSIFD.GPSLatitudeRef : lat_ref,
                piexif.GPSIFD.GPSLatitude : lat_deg,
                piexif.GPSIFD.GPSLongitudeRef : lon_ref,
                piexif.GPSIFD.GPSLongitude : lon_ref,
            }
        }

        exif_bytes = piexif.dump(exif_dict)
        image.save(filename,exif=exif_bytes)

        self.get_logger().info(f"Imagen guardada con latitud {self.latest_lat} y longitud {self.latest_lon}: {filename}")
        self.count_photo += 1  # Incrementar el contador de imágenes


def main(args=None):
    rclpy.init(args=args)
    node = PhotoOutdoor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
