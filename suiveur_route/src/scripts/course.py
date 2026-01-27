#!/usr/bin/env python3

## Code a lancer avant de lancer ./runlaunch suiveur_route runC.launch 
# sed -i 's/\r$//' /home/ubuntu/catkin_ws/src/suiveur_route/src/scripts/course.py
# sudo apt-get install dos2unix
# dos2unix /home/ubuntu/catkin_ws/src/suiveur_route/src/scripts/course.py

# lancer une fois chmod +x start_course.sh
# pour rapide lancer ./start_course.sh


import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class CourseRobot:
    def __init__(self):
        rospy.init_node('course_node')
        
        # CONFIGURATION ROBOT
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.LINEAR_SPEED = 0.5
        self.ANGULAR_SPEED = 0.8
        
        # CONFIGURATION VIDEO
        self.bridge = CvBridge()
        self.video_writer = None
        self.filename = "video_course.avi"
        self.fps = 20.0
        
        # CONFIGURATION TEMPORISATION (POUR FILMER PLUS LONGTEMPS)
        self.stop_requested = False
        self.stop_start_time = 0
        self.EXTRA_RECORD_TIME = 3.0 # Nombre de secondes a filmer apres l'arret
        
        # SUBSCRIBER
        # On n'ecoute plus que la camera
        self.sub_cam = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.camera_callback)

        rospy.loginfo("=== MODE COURSE ===")
        rospy.loginfo("1. Cliquez sur la fenetre video 'Vue Robot'")
        rospy.loginfo("2. Utilisez Z (Avance), S (Recule), Q (Gauche), D (Droite)")
        rospy.loginfo("3. ESPACE pour Arreter")

    def camera_callback(self, msg):
        try:
            # Conversion ROS -> OpenCV
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Gestion de l'enregistrement video
            if self.video_writer is None:
                height, width, _ = frame.shape
                fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                self.video_writer = cv2.VideoWriter(self.filename, fourcc, self.fps, (width, height))
            
            self.video_writer.write(frame)
            
            # Gestion de la temporisation de fin
            if self.stop_requested:
                # On ajoute du texte sur l'image pour dire qu'on finit d'enregistrer
                cv2.putText(frame, "ENREGISTREMENT FINAL...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Verification du temps ecoule
                elapsed_time = time.time() - self.stop_start_time
                if elapsed_time > self.EXTRA_RECORD_TIME:
                    rospy.signal_shutdown("Fin de l'enregistrement supplementaire")
            
            # Affichage et CONTROLE CLAVIER
            # On affiche l'image pour pouvoir cliquer dessus
            cv2.imshow("Vue Robot (Cliquez ici pour piloter)", frame)
            
            # cv2.waitKey(1) attend une touche pendant 1ms
            # & 0xFF permet de recuperer le code ASCII proprement
            key = cv2.waitKey(1) & 0xFF
            
            # Si une touche est detectee (255 veut dire "rien")
            if key != 255: 
                self.gerer_commande(key)
                
        except Exception as e:
            rospy.logerr(f"Erreur : {e}")

    def gerer_commande(self, key):
        lin = self.twist.linear.x
        ang = self.twist.angular.z
        
        # Codes ASCII (minuscules)
        # z=122, s=115, q=113, d=100, espace=32, echap=27
        
        if key == ord('z'): # Avancer
            lin = self.LINEAR_SPEED
            ang = 0.0
            rospy.loginfo("AVANCER")
            
        elif key == ord('s'): # Reculer
            lin = -self.LINEAR_SPEED
            ang = 0.0
            rospy.loginfo("RECULER")
            
        elif key == ord('q'): # Gauche
            ang = self.ANGULAR_SPEED
            lin = 0.0
            rospy.loginfo("GAUCHE")
            
        elif key == ord('d'): # Droite
            ang = -self.ANGULAR_SPEED
            lin = 0.0
            rospy.loginfo("DROITE")
            
#        elif key == 32: # Espace -> STOP
#            lin = 0.0
#            ang = 0.0
#            rospy.loginfo("STOP")
            
        elif key == 27: # Echap -> Quitter avec delai
            rospy.loginfo(f"Arret demande... Continuation de l'enregistrement pour {self.EXTRA_RECORD_TIME} secondes.")
            # On arrete le robot
            lin = 0.0
            ang = 0.0
            # On active le flag d'arret differe
            self.stop_requested = True
            self.stop_start_time = time.time()

        # Envoi de la commande au robot
        self.twist.linear.x = lin
        self.twist.angular.z = ang
        self.cmd_pub.publish(self.twist)

    def cleanup(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_pub.publish(self.twist)
        
        if self.video_writer is not None:
            self.video_writer.release()
            
        cv2.destroyAllWindows()
        rospy.loginfo("Arret du robot et sauvegarde video terminee.")

if __name__ == '__main__':
    try:
        course = CourseRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'course' in locals():
            course.cleanup()