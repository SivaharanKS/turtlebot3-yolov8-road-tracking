#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from ultralytics import YOLO
from collections import deque

class DetecteurNavigation:
    def __init__(self):
        rospy.init_node('detecteur_nav_node')
        self.bridge = CvBridge()

        try:
            self.model = YOLO("pro.onnx", task='segment')
            rospy.loginfo("Modele ONNX charge.")
        except:
            self.model = YOLO("pro.pt", task='segment')
            rospy.loginfo("Modele PT charge.")

        self.cmd_pub = rospy.Publisher('/nav_info', Point, queue_size=1)

        self.subscriber = rospy.Subscriber(
            '/raspicam_node/image/compressed',
            CompressedImage,
            self.camera_callback,
            queue_size=1,
            buff_size=2**24
        )

        self.IMG_SIZE = 128

        # --- CLASSES ---
        self.CLS_ROUTE = 10
        self.CLS_PIETON = 8
        self.CLS_FLECHE = 4
        self.CLS_FLECHE_R = 5
        self.CLS_DOUBLEFLECHE = 2
        self.CLS_DOUBLEFLECHE_R = 3
        self.CLS_ROBOT = 9
        self.CLS_PARKING = 7
        self.CLS_LIMITE = 6

         # DEFINITION DES PRIORITES (Plus le chiffre est grand, plus c'est important)
        self.PRIORITY_MAP = {
            self.CLS_PIETON: 100,
            self.CLS_FLECHE_R: 90,    # Sens Interdit
            self.CLS_DOUBLEFLECHE_R: 90,
            self.CLS_FLECHE: 80,      # Virage
            self.CLS_DOUBLEFLECHE: 80,
            self.CLS_ROBOT: 70,
            self.CLS_PARKING: 60,
            self.CLS_ROUTE: 10,       # Priorite basse (par defaut)
            self.CLS_LIMITE: 0        # On ignore la limite pour l'action Z
        }

        # <<< MODIF >>> seuils par classe
        self.SEUILS_AREA = {
            self.CLS_PIETON: 6000,
            self.CLS_FLECHE: 2000,
            self.CLS_DOUBLEFLECHE: 2000,
            self.CLS_FLECHE_R: 2500,
            self.CLS_DOUBLEFLECHE_R: 2500,
            self.CLS_ROBOT: 3000,
            self.CLS_PARKING: 2500
        }

        # <<< MODIF >>> memoire anti-clignotement
        self.action_buffer = deque(maxlen=5)

    def camera_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            frame = cv2.resize(frame, (self.IMG_SIZE, self.IMG_SIZE))
            self.analyser_image(frame)
        except Exception as e:
            rospy.logwarn_throttle(2, f"Erreur vision: {e}")
           
        # Pour visualiser la camera  
#        try:
#            # Conversion ROS -> OpenCV
#            frame_orig = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
#            
#            # Resize
#            frame = cv2.resize(frame_orig, (self.IMG_SIZE, self.IMG_SIZE))
#            
#            # Prediction
#            # On lance la prediction.
#            # Note : imgsz=128 est important pour la vitesse
#            results = self.model.predict(frame, imgsz=self.IMG_SIZE, conf=0.5, verbose=False)
#            
#            # Visualisation
#            final_frame = frame
#            if len(results) > 0:
#                # La methode plot() gere automatiquement les masques si le modele est de type segment
#                final_frame = results[0].plot()
#
#            # Affichage
#            cv2.imshow("detection", final_frame)
#                
#        except Exception as e:
#            rospy.logerr(f"Erreur callback : {e}")

    def analyser_image(self, frame):
        results = self.model.predict(frame, imgsz=self.IMG_SIZE, conf=0.4, verbose=False)

        target_msg = Point()
        target_msg.z = float(self.CLS_ROUTE)
        target_msg.x = 0.0
        target_msg.y = 0.0

        road_center_x = self.IMG_SIZE // 2

        if len(results) == 0:
            self.cmd_pub.publish(target_msg)
            return

        r = results[0]

        # --- ROUTE ---
        if r.masks is not None:
            # On ne fait le plot que si on a besoin de debug (economie CPU)
            # annotated_frame = r.plot()
            mask_route = np.zeros((self.IMG_SIZE, self.IMG_SIZE), dtype=np.uint8)
            for i, cls_id in enumerate(r.boxes.cls):
                if int(cls_id) == self.CLS_ROUTE:
                    pts = r.masks.xy[i].astype(np.int32)
                    cv2.fillPoly(mask_route, [pts], 255)

            mask_bas = mask_route[self.IMG_SIZE//2:, :]
            M = cv2.moments(mask_bas)
            if M["m00"] != 0:
                road_center_x = int(M["m10"] / M["m00"])

        # --- OBJETS ---
        best_priority = -1
        best_action = self.CLS_ROUTE
        best_area = 0.0

        for box in r.boxes:
            cls_id = int(box.cls)
            conf = float(box.conf)

            # <<< MODIF >>> filtrage confiance
            if conf < 0.6:
                continue

            # <<< MODIF >>> pieton plus strict
            if cls_id == self.CLS_PIETON and conf < 0.75:
                continue

            if cls_id not in self.PRIORITY_MAP or cls_id == self.CLS_ROUTE:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)

            seuil = self.SEUILS_AREA.get(cls_id, 99999)

            if area < seuil:
                continue

            priority = self.PRIORITY_MAP[cls_id]
            if priority > best_priority:
                best_priority = priority
                best_action = cls_id
                best_area = area

        # <<< MODIF >>> lissage temporel
        self.action_buffer.append(best_action)
        if self.action_buffer.count(best_action) < 3:
            best_action = self.CLS_ROUTE

        err_x = road_center_x - (self.IMG_SIZE // 2)

        target_msg.x = float(err_x)
        target_msg.y = float(best_area)
        target_msg.z = float(best_action)

        self.cmd_pub.publish(target_msg)
       
#    def cleanup(self):
#        cv2.destroyAllWindows()

if __name__ == '__main__':
    DetecteurNavigation()
    rospy.spin()
#   try:
#       DetecteurNavigation()
#       rospy.spin()
#   except rospy.ROSInterruptException:
#       pass
#   finally:
#       if 'robot' in locals():
#           robot.cleanup()
