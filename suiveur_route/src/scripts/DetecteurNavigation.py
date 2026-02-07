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
        
#        #===============================================================
#        # 1er dataset
#        #===============================================================
#        
#        # Noms des classes dans le DATA.YAML
#        self.CLASS_ROUTE = ["route-n4RP", "route-GSnu"] 
#        self.CLASS_PIETON = "pieton"
#        self.CLASS_FLECHE = "fleche"
#        self.CLASS_ROBOT = "robot"
#        self.CLASS_PARKING = "parking"
#        
#        # Noms des fleches demi-tour
#        self.CLASS_FLECHE_R = ["fleche_r", "flech_r"]     # Fleche simple envers
#        self.CLASS_D_FLECHE_R = "d_fleche_r" # Double fleche envers
#        self.CLASS_D_FLECHE = "d_fleche"     # Double fleche

        #===============================================================
        # 2e dataset
        #===============================================================

        # Noms des classes dans le DATA.YAML
        self.CLASS_ROUTE = "Route_detect" 
        self.CLASS_PIETON = "passage_pieton"
        self.CLASS_FLECHE = "Fleche_simple"
        self.CLASS_ROBOT = "robot"
        self.CLASS_PARKING = "parking"
        
        # Noms des fleches demi-tour
        self.CLASS_FLECHE_R = "inverse"     # Fleche simple envers
        self.CLASS_D_FLECHE_R = "inverse" # Double fleche envers
        self.CLASS_D_FLECHE = "fleche_double"     # Double fleche

        
        # REGLAGE DE LA DISTANCE (Taille en pixels)
        # Image 128x128 = 16384 pixels total.
        # Ajuste cette valeur : Si il s'arrete trop tot, AUGMENTE la valeur.
        self.SEUIL_PIETON_PROCHE = 2000
        self.SEUIL_FLECHE_PROCHE = 350 
        self.SEUIL_ROBOT_PROCHE = 2000
#        self.SEUIL_UTURN_PROCHE = 350 # Seuil pour declencher le demi-tour 
        self.SEUIL_DOUBLE_PROCHE = 500
        self.SEUIL_PARKING_PROCHE = 800
        
        # Anti-Latence : Flag pour ne pas traiter si occupe
        self.processing = False 

#        #===============================================================
#        # 1er dataset
#        #===============================================================
#
#        # CHARGEMENT MODELE
#        try:
#            # On charge le modele
#            self.model = YOLO('pro.onnx', task='segment')
#            rospy.loginfo("Modele ONNX charge (Input: 128x128).")
#        except Exception as e:
#            rospy.logerr(f"Erreur chargement modele: {e}")
#            rospy.signal_shutdown("Modele introuvable")
            
        #===============================================================
        # 2e dataset
        #===============================================================
            
        # CHARGEMENT MODELE
        try:
            # On charge le modele
            self.model = YOLO('pr.onnx', task='segment')
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
        # Gestion de la latence
        if self.processing:
            return
        self.processing = True

        try:
            # Conversion ROS -> OpenCV
            frame_orig = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Resize (CRITIQUE pour le ONNX)
            frame = cv2.resize(frame_orig, (self.IMG_SIZE, self.IMG_SIZE))
            
            # Prediction YOLO
            results = self.model.predict(frame, imgsz=self.IMG_SIZE, conf=0.5, verbose=False)
            result = results[0]

            nav_msg = Point()
            nav_msg.z = 0.0 # Par defaut: pas de route
            
            img_center_x = self.IMG_SIZE // 2
            
            # Dessin (Genere l'image avec masques mais ne l'affiche pas encore)
            final_frame = result.plot() 

            # Analyse des masques
            if result.masks is not None and len(result.masks.data) > 0:
                
                class_ids = result.boxes.cls.cpu().numpy().astype(int)
                
                # Variables pour stocker 
                # ROUTE
                route_center = None # Variable pour le route
                route_area = 0
                
                # PIETON
                pieton_center = None # Variable pour le pieton
                pieton_area = 0
                
                # FLECHE
                fleche_center = None # Variable pour le fleche
                fleche_area = 0
                
                # ROBOT
                robot_center = None # Variable pour le robot
                robot_area = 0
                
#                # DEMI-TOUR
#                uturn_center = None # Pour le demi-tour
#                uturn_area = 0
                
                # DOUBLE FLECHE
                d_fleche_center = None 
                d_fleche_area = 0
                
                # PARKING
                parking_center = None
                parking_area = 0                
                
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
                      
                      # On divise par 255 pour avoir le nombre de pixels
                      area = M["m00"] / 255.0

                      if name == self.CLASS_PIETON:
                        if area > pieton_area:
                          pieton_area = area
                          pieton_center = (cx, cy)
                          
                      elif name == self.CLASS_FLECHE:
                        if area > fleche_area:
                          fleche_area = area
                          fleche_center = (cx, cy)
                          
                      elif name == self.CLASS_ROBOT:
                            if area > robot_area:
                                robot_area = area
                                robot_center = (cx, cy)  
                                
                      elif name == self.CLASS_ROUTE:
#                      elif name in self.CLASS_ROUTE:
                        if area > route_area:
                          route_area = area
                          route_center = (cx, cy)     
                          
#                      # Detection Demi-Tour 
#                      elif name in self.CLASS_FLECHE_R or name == self.CLASS_D_FLECHE_R:
#                        if area > uturn_area:
#                          uturn_area = area
#                          uturn_center = (cx, cy)
                          
                      # Double fleche normale
                      elif name == self.CLASS_D_FLECHE:
                        if area > d_fleche_area: 
                          d_fleche_area = area 
                          d_fleche_center = (cx, cy)
                      
                      # PARKING
                      elif name == self.CLASS_PARKING:
                        if area > parking_area:  
                          parking_area = area
                          parking_center = (cx, cy)
                          
#                # ==========================================================
#                # ZONE DE DEBUG FLECHE
#                # ==========================================================
#                if fleche_center is not None:
#                    # Affiche la taille de la fleche dans le terminal (2 fois par seconde max)
#                    rospy.loginfo_throttle(0.5, f"DEBUG FLECHE -> Taille vue: {fleche_area:.0f} (Seuil requis: {self.SEUIL_FLECHE_PROCHE})")
#                # ==========================================================         
                
#                # ==========================================================
#                # ZONE DE DEBUG ROBOT
#                # ==========================================================
#                if robot_center is not None:
#                    # Affiche la taille vue dans le terminal
#                    rospy.loginfo_throttle(0.5, f"DEBUG ROBOT -> Taille vue: {robot_area:.0f} (Seuil requis: {self.SEUIL_ROBOT_PROCHE})")
#                # ==========================================================

#                # ==========================================================
#                # ZONE DE DEBUG DOUBLE FLECHE
#                # ==========================================================
#                if d_fleche_center is not None:
#                    # Affiche la taille vue dans le terminal
#                    rospy.loginfo_throttle(0.5, f"DEBUG DOUBLE FLECHE -> Taille vue: {d_fleche_area:.0f} (Seuil requis: {self.SEUIL_DOUBLE_PROCHE})")
#                # ==========================================================

#                # ==========================================================
#                # ZONE DE DEBUG PARKING
#                # ==========================================================
#                if parking_area is not None:
#                    # Affiche la taille vue dans le terminal
#                    rospy.loginfo_throttle(0.5, f"DEBUG PARKING -> Taille vue: {parking_area:.0f} (Seuil requis: {self.SEUIL_FLECHE_PROCHE})")
#                # ==========================================================

                # LOGIQUE DE PRIORITE AVEC DISTANCE (PIXEL)
                found_action = False  # On initialise la variable avant les if
                     
                # PRIORITE 1 : DOUBLE FLECHE (Detection pour Memoire) - Code 6
                # Si le pieton est loin, on verra la fleche -> Code 6 -> Memoire activee.
                # Quand le pieton sera proche -> Code 2 -> Arret + Verification Memoire.
                if not found_action and d_fleche_center is not None and d_fleche_area > self.SEUIL_DOUBLE_PROCHE:
                    cx, cy = d_fleche_center
                    cv2.circle(final_frame, (cx, cy), 15, (128, 0, 128), -1) 
                    cv2.putText(final_frame, "MEMOIRE D_FLECHE", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128,0,128), 2)
                    
                    nav_msg.x = float(cx - img_center_x) # On envoie la position pour le suivi si besoin
                    nav_msg.y = float(d_fleche_area)
                    nav_msg.z = 6.0 
                    
                    # ASTUCE : Pour ne pas perdre le fil, si on voit aussi la route, on peut aider le controleur
                    if route_center is not None:
                         nav_msg.x = float(route_center[0] - img_center_x)

                    found_action = True     
                                                     
                # PRIORITE 2 : PIETON 
                # Verification du Pieton ET de sa taille
                elif pieton_center is not None and pieton_area > self.SEUIL_PIETON_PROCHE:
                    # Il est PROCHE -> ARRET
                    cx, cy = pieton_center
                    cv2.circle(final_frame, (cx, cy), 10, (0, 0, 255), -1)
                    
                    nav_msg.x = float(cx - img_center_x)
                    nav_msg.y = float(pieton_area)
                    nav_msg.z = 2.0 # STOP
                    found_action = True
                    
                # PRIORITE 3 : OBSTACLE ROBOT 
                # Si on voit un robot ET qu'il est proche -> ON S'ARRETE
                elif not found_action and robot_center is not None and robot_area > self.SEUIL_ROBOT_PROCHE:
                    cx, cy = robot_center
                    cv2.circle(final_frame, (cx, cy), 10, (0, 0, 0), -1) # Point Noir
                    cv2.putText(final_frame, "OBSTACLE", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
                    
                    nav_msg.x = float(cx - img_center_x)
                    nav_msg.y = float(robot_area)
                    nav_msg.z = 4.0 # ROBOT
                    found_action = True
                    
#                # PRIORITE 4 : DEMI-TOUR (180 degres)
#                elif not found_action and uturn_center is not None and uturn_area > self.SEUIL_UTURN_PROCHE:
#                    cx, cy = uturn_center
#                    cv2.circle(final_frame, (cx, cy), 10, (0, 255, 255), -1) 
#                    cv2.putText(final_frame, "DEMI-TOUR", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
#                    
#                    nav_msg.z = 5.0 # DEMI-TOUR
#                    found_action = True   
                   
                # PRIORITE 5 : PARKING    
                elif not found_action and parking_center is not None and parking_area > self.SEUIL_PARKING_PROCHE:
                    cx, cy = parking_center
                    cv2.circle(final_frame, (cx, cy), 15, (0, 255, 0), -1)
                    cv2.putText(final_frame, "PARKING VU", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                    
                    nav_msg.x = float(cx - img_center_x) # Erreur par rapport au centre 
                    nav_msg.y = float(parking_area) # Taille pour savoir quend entrer
                    nav_msg.z = 7.0 # Code PARKING
                    
                    found_action = True                  
                    
                # PRIORITE 6 : FLECHE 
                elif not found_action and fleche_center is not None and fleche_area > self.SEUIL_FLECHE_PROCHE:
                    # On voit la fleche ET on est proche
                    cx_f, cy_f = fleche_center
                    cv2.circle(final_frame, (cx_f, cy_f), 10, (255, 0, 0), -1)
                    nav_msg.z = 3.0 # FLECHE
                    
                    # MAIS on envoie la position de la ROUTE pour le guidage
                    if route_center is not None:
                        cx_r, cy_r = route_center
                        nav_msg.x = float(cx_r - img_center_x)
                        # Visuel : Trait vers la route meme en mode fleche
                        cv2.line(final_frame, (img_center_x, cy_r), (cx_r, cy_r), (255, 0, 255), 3)
                    else:
                        # Si on voit la fleche mais pas la route (rare), on vise la fleche par defaut
                        nav_msg.x = float(cx_f - img_center_x)

                    found_action = True

                # PRIORITE 7 : ROUTE NORMALE
                if not found_action and route_center is not None:
                    cx, cy = route_center
                    cv2.circle(final_frame, (cx, cy), 5, (0, 255, 0), -1)
                    cv2.line(final_frame, (img_center_x, cy), (cx, cy), (255, 255, 0), 2)
                    
                    nav_msg.x = float(cx - img_center_x)
                    nav_msg.y = float(route_area)
                    nav_msg.z = 1.0 # ROUTE
            
            # 7. Publication des ordres au moteur
            self.nav_pub.publish(nav_msg)

            # =========================================================
            # PARTIE VISUELLE (COMMENTEE POUR PERFORMANCES)
            # Decommentez les lignes ci-dessous pour voir l'image
            # =========================================================
            
            display_frame = cv2.resize(final_frame, (512, 512), interpolation=cv2.INTER_NEAREST)
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