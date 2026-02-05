#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO

class DetecteurNavigation:
    def __init__(self):
        rospy.init_node('detecteur_nav_node')
       
        # CONFIGURATION VISION
        self.bridge = CvBridge()
        self.IMG_SIZE = 128  # Taille exigee par le modele ONNX
       
        # Noms des classes dans le DATA.YAML
        self.CLASS_ROUTE = "route-GSnu"
        self.CLASS_PIETON = "pieton"
       
        # REGLAGE DE LA DISTANCE (Taille en pixels)
        # Image 128x128 = 16384 pixels total.
        # 1000 = pieton moyen
        # 2500 = pieton tres proche
        # Ajuste cette valeur : Si il s'arrete trop tot, AUGMENTE la valeur.
        self.SEUIL_PIETON_PROCHE = 2000
       
        # Anti-Latence : Flag pour ne pas traiter si occupe
        self.processing = False

        # CHARGEMENT MODELE
        try:
            # On charge le modele
            self.model = YOLO('pro.onnx', task='segment')
            rospy.loginfo("Modele ONNX charge (Input: 128x128).")
        except Exception as e:
            rospy.logerr(f"Erreur chargement modele: {e}")
            rospy.signal_shutdown("Modele introuvable")

        # PUBLISHERS
        # Envoie les coordonnees (x, y, z) au ControleurNavigation
        self.nav_pub = rospy.Publisher('/navigation/info', Point, queue_size=1)
       
        # Publisher image (Optionnel pour debug a distance)
        self.img_pub = rospy.Publisher('/navigation/image/compressed', CompressedImage, queue_size=1)

        # SUBSCRIBER
        self.sub_cam = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.camera_callback, queue_size=1, buff_size=2**24)


    def camera_callback(self, msg):
        # 1. Gestion de la latence
        if self.processing:
            return
        self.processing = True

        try:
            # 2. Conversion ROS -> OpenCV
            frame_orig = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
           
            # 3. Resize (CRITIQUE pour le ONNX)
            frame = cv2.resize(frame_orig, (self.IMG_SIZE, self.IMG_SIZE))
           
            # 4. Prediction YOLO
            results = self.model.predict(frame, imgsz=self.IMG_SIZE, conf=0.5, verbose=False)
            result = results[0]

            nav_msg = Point()
            nav_msg.z = 0.0 # Par defaut: pas de route
           
            img_center_x = self.IMG_SIZE // 2
           
            # 5. Dessin (Genere l'image avec masques mais ne l'affiche pas encore)
            final_frame = result.plot()

            # 6. Analyse des masques
            if result.masks is not None:
               
                class_ids = result.boxes.cls.cpu().numpy().astype(int)
               
                # Variables pour stocker la meilleure route et le meilleur pieton
                route_center = None
                route_area = 0
               
                pieton_center = None
                pieton_area = 0
               
                # Analyse de tous les objets detectes
                for i, class_id in enumerate(class_ids):
                    name = result.names[class_id]
                   
                    # Masque binaire
                    mask_raw = result.masks.data[i].cpu().numpy()
                    mask_bin = (mask_raw > 0.5).astype(np.uint8) * 255
                    mask_bin = cv2.resize(mask_bin, (self.IMG_SIZE, self.IMG_SIZE))

                    # Calcul du centre
                    M = cv2.moments(mask_bin)
                    if M["m00"] > 0:
                      cx = int(M["m10"] / M["m00"])
                      cy = int(M["m01"] / M["m00"])
                     
                      #On divise par 255 pour avoir le vrai nombre de pixels
                      area = M["m00"] / 255.0

                      if name == self.CLASS_PIETON:
                        if area > pieton_area:
                          pieton_area = area
                          pieton_center = (cx, cy)
                               
                      elif name == self.CLASS_ROUTE:
                        if area > route_area:
                          route_area = area
                          route_center = (cx, cy)              

                # LOGIQUE DE PRIORITE AVEC DISTANCE
               
                stop_pieton = False
               
                # 1. Verification du Pieton ET de sa taille
                if pieton_center is not None:
                    # Pour t'aider a regler, j'affiche la taille dans le terminal
                    # rospy.loginfo(f"Pieton vu. Taille: {pieton_area:.0f} / Seuil: {self.SEUIL_PIETON_PROCHE}")
                   
                    if pieton_area > self.SEUIL_PIETON_PROCHE:
                        # Il est PROCHE -> ARRET
                        cx, cy = pieton_center
                        cv2.circle(final_frame, (cx, cy), 10, (0, 0, 255), -1)
                        cv2.putText(final_frame, "STOP PIETON", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                       
                        nav_msg.x = float(cx - img_center_x)
                        nav_msg.y = float(pieton_area)
                        nav_msg.z = 2.0 # Code STOP
                        stop_pieton = True
                    else:
                        # Il est LOIN -> On l'ignore (on le dessine juste en orange pour info)
                        cx, cy = pieton_center
                        cv2.circle(final_frame, (cx, cy), 5, (0, 165, 255), -1)
                        # On ne met pas stop_pieton a True, donc on ira voir la route
               
                # 2. Si pas de STOP Pieton, on suit la route
                if not stop_pieton and route_center is not None:
                    cx, cy = route_center
                    cv2.circle(final_frame, (cx, cy), 5, (0, 255, 0), -1)
                    cv2.line(final_frame, (img_center_x, cy), (cx, cy), (255, 255, 0), 2)
                   
                    nav_msg.x = float(cx - img_center_x)
                    nav_msg.y = float(route_area)
                    nav_msg.z = 1.0 # Code Route
           
            # 7. Publication des ordres au moteur
            self.nav_pub.publish(nav_msg)

            # =========================================================
            # PARTIE VISUELLE (COMMENTEE POUR PERFORMANCES)
            # Decommentez les lignes ci-dessous pour voir l'image
            # =========================================================
           
            cv2.imshow("Vue Robot", final_frame)
            cv2.waitKey(1)

            # =========================================================
               
        except Exception as e:
            rospy.logerr(f"Erreur callback : {e}")
           
        finally:
            self.processing = False

    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = DetecteurNavigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'node' in locals():
            node.cleanup()
