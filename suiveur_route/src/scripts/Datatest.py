#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO

class DataTestRobot:
    def __init__(self):
        rospy.init_node('datatest_node')
        
        # CONFIGURATION ROBOT
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.LINEAR_SPEED = 0.5
        self.ANGULAR_SPEED = 0.8
        
        # CONFIGURATION VISION
        self.bridge = CvBridge()
        self.IMG_SIZE = 128 
        
        # CHARGEMENT MODELE
        try:
            # On charge le fichier .pt directement. 
            self.model = YOLO('pr.onnx', task='segment') 

        except Exception as e:
            rospy.logerr(f"Erreur chargement modele: {e}")
            rospy.signal_shutdown("Modele introuvable")

        # SUBSCRIBER
        self.sub_cam = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.camera_callback)
        rospy.loginfo("Utilisez ZQSD sur la fenetre pour piloter.")

    def camera_callback(self, msg):
        try:
            # Conversion ROS -> OpenCV
            frame_orig = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Resize
            frame = cv2.resize(frame_orig, (self.IMG_SIZE, self.IMG_SIZE))
            
            # Prediction
            # On lance la prediction. 
            # Note : imgsz=128 est important pour la vitesse
            results = self.model.predict(frame, imgsz=self.IMG_SIZE, conf=0.5, verbose=False)
            
            # Visualisation
            final_frame = frame
            if len(results) > 0:
                # La methode plot() gere automatiquement les masques si le modele est de type segment
                final_frame = results[0].plot()

            # Affichage
            cv2.imshow("Test Dataset (ZQSD)", final_frame)
            
            # Controle
            key = cv2.waitKey(1) & 0xFF
            if key != 255: 
                self.gerer_commande(key)
                
        except Exception as e:
            rospy.logerr(f"Erreur callback : {e}")

    def gerer_commande(self, key):
        lin = 0.0
        ang = 0.0
        
        if key == ord('z'): lin = self.LINEAR_SPEED
        elif key == ord('s'): lin = -self.LINEAR_SPEED
        elif key == ord('q'): ang = self.ANGULAR_SPEED
        elif key == ord('d'): ang = -self.ANGULAR_SPEED
        elif key == 32: pass # Espace = Stop
        elif key == 27: rospy.signal_shutdown("Arret")

        self.twist.linear.x = lin
        self.twist.angular.z = ang
        self.cmd_pub.publish(self.twist)

    def cleanup(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_pub.publish(self.twist)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        robot = DataTestRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'robot' in locals():
            robot.cleanup()
