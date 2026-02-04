#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist, Point

class ControleurNavigation:
    def __init__(self):
        rospy.init_node('controleur_nav_node')
        self.rate = rospy.Rate(10)
       
        self.velocity = Twist()
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub_info = rospy.Subscriber('/nav_info', Point, self.callback_info)
       
        # Donnees recues du detecteur
        self.action_id = -1
        self.err_x = 0.0     # Erreur de centrage sur la route
        self.obj_size = 0.0  # Taille de l'objet (Proximite)
       
        # Gestion des etats
        self.is_waiting = False
        self.wait_start_time = 0
       
        # Memoire pour ne pas rester bloque (Cooldown)
        self.last_event_time = 0
        self.COOLDOWN_TIME = 2.0 # Temps avant de reagir au meme panneau
       
        # --- REGLAGES A CALIBRER SUR LE TERRAIN ---
        # SEUIL DE DISTANCE : Plus c'est grand, plus il faut etre pres pour reagir
        # Essayez entre 6000 et 15000 selon la resolution de votre camera
        self.SEUIL_PROXIMITE = 8000.0
       
        # Gains et Vitesses
        self.K_ROT_NORMAL = 0.005  # Suivi de ligne doux (Ligne droite)
        self.K_ROT_VIRAGE = 0.012  # Suivi de ligne agressif (Virage)
       
        self.V_NORMAL = 0.20       # Vitesse de croisiere
        self.V_LENTE = 0.08        # Vitesse d'approche / Virage
        self.V_ROTATION_180 = 1.0  # Vitesse pour faire demi-tour
        self.TIME_180 = 3.2        # Temps pour faire 180 degre

    def callback_info(self, msg):
        self.err_x = msg.x      
        self.obj_size = msg.y  
        self.action_id = int(msg.z)

    def run(self):
        while not rospy.is_shutdown():
           
            # 1. SECURITE : Attente du demarrage du detecteur
            if self.action_id == -1:
                rospy.loginfo_throttle(2, "Attente vision...")
                self.drive(0, 0)
                self.pub_vel.publish(self.velocity)
                self.rate.sleep()
                continue

            # 2. GESTION DE L'ARRET TEMPORAIRE (Pieton)
            if self.is_waiting:
                # On attend 2 secondes comme demande
                if time.time() - self.wait_start_time > 2.0:
                    self.is_waiting = False
                    self.last_event_time = time.time() # Active le cooldown
                    rospy.loginfo("Fin d'arret -> J'avance")
                else:
                    self.drive(0, 0)
                    self.pub_vel.publish(self.velocity)
                    self.rate.sleep()
                    continue

            # 3. ANALYSE DE LA SITUATION
           
            # Est-ce que l'objet est assez proche pour qu'on reagisse ?
            is_close = self.obj_size > self.SEUIL_PROXIMITE
           
            # Est-ce qu'on est en "Cooldown" (on vient de reagir, on ne le refait pas tout de suite)?
            in_cooldown = (time.time() - self.last_event_time) < self.COOLDOWN_TIME

            # --- DECISION ---
           
            lin = 0.0
            ang = 0.0
           
            # PAR DEFAUT : On suit la route (C'est la regle d'or)
            # On applique la correction pour rester au centre
            target_speed = self.V_NORMAL
            target_k = self.K_ROT_NORMAL
           
            # --- CAS PRIORITAIRES (S'activent seulement si PROCHE) ---
           
            # A. PIETON (ID 8)
            if self.action_id == 8 and is_close and not in_cooldown:
                rospy.loginfo("PIETON PROCHE -> STOP 2s")
                self.is_waiting = True
                self.wait_start_time = time.time()
                lin = 0.0
                ang = 0.0 # On s'arrete
               
                # On envoie la commande et on continue la boucle (pour gerer le waiting au prochain tour)
                self.drive(lin, ang)
                self.pub_vel.publish(self.velocity)
                self.rate.sleep()
                continue

            # B. FLECHE / VIRAGE (ID 4)
            elif self.action_id == 4 and is_close:
                # "Tourne petit a petit" -> On ralentit et on force le suivi de ligne agressif
                target_speed = self.V_LENTE
                target_k = self.K_ROT_VIRAGE

            # C. DEMI-TOUR (ID 3 ou 5 - Fleche_R / Sens Interdit)
            elif (self.action_id == 3 or self.action_id == 5) and is_close and not in_cooldown:
                rospy.loginfo("SENS INTERDIT PROCHE -> 180 degre")
                self.faire_demi_tour()
                # Apres le demi-tour, on reset tout pour repartir propre
                self.action_id = 10
                self.last_event_time = time.time()
                continue # On recommence la boucle

            # D. ROBOT / PARKING (ID 9 ou 7)
            elif (self.action_id == 9 or self.action_id == 7) and is_close:
                target_speed = 0.0 # Stop simple tant que l'obstacle est la

           
            # --- APPLICATION DE LA COMMANDE (Suivi de Route) ---
            # C'est ici qu'on assure que le robot reste toujours sur la route
            # Si aucune priorite ne l'a arrete, il avance en corrigeant sa trajectoire
           
            lin = target_speed
            # La correction angulaire est toujours active pour centrer le robot
            ang = -self.err_x * target_k
           
            # <<< MODIF >>> saturation angulaire
            ang = max(min(ang, 1.0), -1.0)

            self.drive(lin, ang)
            self.pub_vel.publish(self.velocity)
            self.rate.sleep()

    def faire_demi_tour(self):
        # Sequence bloquante pour le 180 degre
        # 1. Petit recul de securite
        self.drive(-0.1, 0)
        self.pub_vel.publish(self.velocity)
        time.sleep(1.0)
       
        # 2. Rotation sur place
        self.drive(0, self.V_ROTATION_180)
        self.pub_vel.publish(self.velocity)
        time.sleep(self.TIME_180)
       
        # 3. Stop
        self.drive(0, 0)
        self.pub_vel.publish(self.velocity)
        time.sleep(0.5)

    def drive(self, x, z):
        self.velocity.linear.x = x
        self.velocity.angular.z = z

if __name__ == '__main__':
    try:
        ctrl = ControleurNavigation()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass
