import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageSaver:
    def __init__(self):
        # Configuración de la carpeta
        self.output_dir = "imagenes_gelsight"
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            print(f"[*] Carpeta creada: {self.output_dir}")

        # Inicialización de herramientas
        self.bridge = CvBridge()
        self.counter = 0
        
        # Suscriptor
        # Nota: anonymous=True permite que el script corra sin conflictos de nombre
        rospy.init_node('image_saver_standalone', anonymous=True)
        self.image_sub = rospy.Subscriber("/gelsight/tactile_image", Image, self.callback)
        
        print("[!] Esperando mensajes en /gelsight/tactile_image...")
        print("[!] Presiona Ctrl+C para detener.")

    def callback(self, data):
        try:
            # Convertimos el mensaje de ROS a una imagen de OpenCV (BGR)
            # El encoding 'bgr8' coincide con tu rostopic echo
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(f"Error en la conversión: {e}")
            return

        # Guardar la imagen
        file_path = os.path.join(self.output_dir, f"frame_{self.counter:04d}.jpg")
        cv2.imwrite(file_path, cv_image)
        
        if self.counter % 10 == 0: # Feedback cada 10 imágenes
            print(f"Guardada imagen {self.counter} en {file_path}")
            
        self.counter += 1

if __name__ == '__main__':
    try:
        saver = ImageSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("\n[!] Programa finalizado por el usuario.")
    except Exception as e:
        print(f"\n[!] Error: {e}")
