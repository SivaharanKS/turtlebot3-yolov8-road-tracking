#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
import random
import time

class ControleurNavigation:
    def __init__(self):
        rospy.init_node('ctl_nav_node')
        self.rate = rospy.Rate(10)
        
        self.velocity = Twist()
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub_info = rospy.Subscriber('/navigation/info', Point, self.callback_nav)
        
        # Etats: 0=Rien, 1=Route, 2=Pieton, 3=Fleche, 4=Robot, 5=Demi-tour, 6=trajectoire
        self.etat = 0
        self.erreur_x = 0.0
        
        # PARAMETRES DE VITESSE
        self.VITESSE_NORMAL = 0.18
        self.VITESSE_RAPIDE = 0.90
        self.TEMPS_MARCHE = 4.5
        self.VITESSE_LENTE  = 0.08  # Vitesse reduite pour bien prendre le virage
        
        # GAIN ROTATION : Comme pour le suivi de balle. 
        # Si ca oscille trop, baisse a 0.010
        # Si ca ne tourne pas assez, monte a 0.020
        self.GAIN_ROTATION = 0.020 
        
        # SEUIL D'ADAPTATION VITESSE (Route)
        # Si l'erreur est superieure a X pixels, on ralentit
        # Image 128px -> Centre a 64. 10 pixels d'erreur c'est deja un petit virage.
        self.SEUIL_ERREUR_VIRAGE = 10.0

        # REGLAGE DU TEMPS D'APPROCHE
        # Combien de temps il avance apres avoir detecte le pieton
        self.TEMPS_APPROCHE = 2  # Essaye 1.0 ou 2.0 selon la distance voulue
        
        # PARAMETRES DEMI-TOUR (A CALIBRER)
        self.VITESSE_ROTATION_180 = 6.2 # Rad/s (assez rapide)
        self.TEMPS_DEMI_TOUR = 4.0      # Temps pour faire 180 degres (3.14 / 1.0 environ)
        
        # 90 degres (Quart de tour droite)
        self.VITESSE_ROTATION_90 = 2.8
        self.TEMPS_ROTATION_90 = 2.0 # A ajuster pour faire 90 degres
#        self.sequence_active = False

        # GESTION ALEATOIRE
        self.memoire_virage = False
        self.choix_random = 0      
        
        # PARKING
        self.phase_parking = 0  # 0=Approche, 1=Entree, 2=Gare
        
        # SECURITE DEMARRAGE (Ajout demande)
        # Le robot ne bouge pas tant qu'il n'a rien vu une premiere fois
        self.active = False

    def callback_nav(self, msg):        
        self.etat = int(msg.z)
        self.erreur_x = msg.x
        self.taille_objet = msg.y

    def run(self):
        while not rospy.is_shutdown():
            try:
                # D'ACTIVATION ===============================================================
                # Si on n'est pas encore active, on regarde si on voit quelque chose
                if not self.active:
                    if self.etat != 0:
                        # On a vu quelque chose (Route, Pieton, Fleche...) ! C'est parti.
                        rospy.loginfo("PREMIERE DETECTION -> ACTIVATION DU ROBOT !")
                        self.active = True
                    else:
                        # Tant qu'on a rien vu, on attend sagement
                        rospy.loginfo_throttle(2, "En attente de detection pour demarrer...")
                        self.drive(0.0, 0.0)
                        self.pub_vel.publish(self.velocity)
                        self.rate.sleep()
                        continue # On passe au tour suivant de boucle
                # ============================================================================                                
                
                # CAS 1 : DOUBLE FLECHE (ALEATOIRE)
                elif self.etat == 6:
                    # On verifie qu'on n'a pas deja pris une decision recemment
                    if not self.memoire_virage:
                        rospy.logwarn("DOUBLE FLECHE VUE -> Memorisation...")
                        self.memoire_virage = True
                        self.choix_random = random.randint(0, 1) # 0=Rien, 1=Virage                    
                        
                        if self.choix_random == 1:
                            rospy.logwarn("DOUBLE FLECHE -> MEMOIRE: VIRAGE PROCHAIN PIETON")
                        else:
                            rospy.logwarn("DOUBLE FLECHE -> MEMOIRE: TOUT DROIT")                                                
                        
                    # On continue de rouler (comme le mode Route)
                    vit_ang = -self.erreur_x * self.GAIN_ROTATION
                    self.drive(self.VITESSE_NORMAL, vit_ang)
                
                # CAS 2 : PIETON DETECTE                  
                elif self.etat == 2:
                    # Approche finale
                    rospy.logwarn("PIETON VU -> STOP...")
                    
                    # Approche et Stop
                    self.drive(self.VITESSE_NORMAL, 0.0)
                    self.pub_vel.publish(self.velocity)
                    rospy.sleep(self.TEMPS_APPROCHE)
                    self.drive(0.0, 0.0)
                    self.pub_vel.publish(self.velocity)
                    rospy.sleep(3.0) 
                    
                    # VERIFICATION MEMOIRE
                    if self.memoire_virage:
                        if self.choix_random == 1:
                            rospy.logwarn("MEMOIRE ACTIVE : ROTATION 90 DROITE !")
                            
                            # ON AVANCE POUR DEPASSER LE PIETON
                            start_time = rospy.get_time()

                            # Allez apres pieton
                            rospy.loginfo("Avance pour degager le passage...")
                            while (rospy.get_time() - start_time) < 2.5: 
                                self.drive(self.VITESSE_RAPIDE, 0.0) # On definit la vitesse
                                self.pub_vel.publish(self.velocity)  
                                self.rate.sleep()                                                  
                            
                            # Rotation
                            self.drive(0.0, -self.VITESSE_ROTATION_90)
                            self.pub_vel.publish(self.velocity)
                            rospy.sleep(self.TEMPS_ROTATION_90)
                            
                            # Stabilisation
                            self.drive(0.0, 0.0)
                            self.pub_vel.publish(self.velocity)
                            rospy.sleep(1.0)
                            
                        else:
                            rospy.loginfo("MEMOIRE ACTIVE : Choix etait 0 -> Tout droit")
                        
                        self.memoire_virage = False
                    
                    # 3. Reprise
                    rospy.loginfo("Reprise...")
                    self.etat = 0
                    
                # CAS 3 : OBSTABLE ROBOT
                elif self.etat == 4:
                    # Ici, pas de sleep. On s'arrete juste pour ce cycle.
                    # Si au prochain cycle le robot est encore la, on reste arrete.
                    # Si il part, l'etat changera et on repartira.
                    rospy.logwarn_throttle(1, "OBSTACLE ROBOT DETECTE -> J'ATTENDS...")
                    self.drive(0.0, 0.0)
                    
                # CAS 4 : DEMI-TOUR
                elif self.etat == 5:
                    rospy.logwarn("ACTION : DEMI-TOUR 180 DEGRES >>>")
                    
                    # On s'arrete 1 sec pour stabiliser
                    self.drive(0.0, 0.0)
                    self.pub_vel.publish(self.velocity)
                    rospy.sleep(1.0)
                    
                    # Rotation sur place (Demi-tour)
                    self.drive(0.0, self.VITESSE_ROTATION_180)
                    self.pub_vel.publish(self.velocity)
                    rospy.sleep(self.TEMPS_DEMI_TOUR)
                    
                    # Stop et reprise
                    self.drive(0.0, 0.0)
                    self.pub_vel.publish(self.velocity)
                    rospy.sleep(1.0)
                    
                    self.etat = 0 # Retour a la normale
                    
                # CAS 5 : FLECHE DETECTE
                elif self.etat == 3:
                    rospy.loginfo_throttle(0.5, "ACTION : VIRAGE DROITE ASSISTE >>>")
                    
                    # On ignore l'erreur X ici pour aller tout droit
                    self.drive(self.VITESSE_LENTE, 0.0) 
                    
#                # CAS 6 : PARKING - SUIT LE PARKING COMME LA ROUTE
#                elif self.etat == 7:
#                    taille_parking = self.taille_objet
#                    
#                    # PHASE 0 : APPROCHE - Suit le parking de loin
#                    if self.phase_parking == 0:
#                        if taille_parking > 100:
#                            self.phase_parking = 1
#                            rospy.logwarn("PARKING PROCHE -> DEBUT ENTREE")                            
#                            # Suit le parking comme la route
#                            rospy.loginfo_throttle(0.5, f"Approche parking (Taille: {taille_parking:.0f})")
#                            vit_ang = -self.erreur_x * self.GAIN_ROTATION
#                            self.drive(self.VITESSE_NORMAL, vit_ang)
#                            
#                            self.drive(self.VITESSE_NORMAL, 0.0)
#                            self.pub_vel.publish(self.velocity)
#                            rospy.sleep(self.TEMPS_APPROCHE)
#                    
#                    # PHASE 1 : ENTREE - Suit le parking pour rentrer dedans
#                    elif self.phase_parking == 1:
#                        # Tant qu'on voit le parking, on le suit
#                        if self.etat == 7:
#                            rospy.loginfo_throttle(0.3, f"Entree parking (Taille: {taille_parking:.0f})")
#                            
#                            # Suit le parking visuellement
#                            vit_ang = -self.erreur_x * self.GAIN_ROTATION
#                            self.drive(self.VITESSE_LENTE, vit_ang)
#                            
#                            # Quand le parking devient TRES grand, on est dedans
#                            if taille_parking > 6500:
#                                rospy.logwarn("PARKING ATTEINT -> ARRET")
#                                self.drive(0.0, 0.0)
#                                self.pub_vel.publish(self.velocity)
#                                rospy.sleep(5.0)
#                                self.phase_parking = 2
#                        else:
#                            # On a perdu le parking
#                            rospy.loginfo_throttle(1, "PERDU -> RECHERCHE (Rotation Droite)")
#                            # Rotation sur place pour chercher la ligne
#                            self.drive(0.0, -0.3)
#                    
#                    # PHASE 2 : GARE - Sortie en sequentiel
#                    elif self.phase_parking == 2:                        
#                        rospy.loginfo("Marche arriere")
#                        self.drive(-0.15, 0.0)
#                        self.pub_vel.publish(self.velocity)
#                        rospy.sleep(3.0)
#                        
#                        rospy.loginfo("Rotation retour")
#                        self.drive(0.0, self.VITESSE_ROTATION_90)
#                        self.pub_vel.publish(self.velocity)
#                        rospy.sleep(self.TEMPS_ROTATION_90)
#                        
#                        rospy.loginfo("Fin parking")
#                        self.phase_parking = 0
#                        self.etat = 0                                 
#                
                # CAS 7 : ROUTE -> AVANCE
                elif self.etat == 1:
                    # Calcul de la rotation normale
                    vit_ang = -self.erreur_x * self.GAIN_ROTATION
                    
                    # Logique adaptative :
                    # Si l'erreur est grande (valeur absolue), on ralentit
                    if abs(self.erreur_x) > self.SEUIL_ERREUR_VIRAGE:
                        rospy.loginfo_throttle(1, "ACTION : Virage (Vitesse Lente)")
                        vit_lin = self.VITESSE_LENTE
                    else:
                        rospy.loginfo_throttle(1, "ACTION : Ligne Droite (Vitesse Normale)")
                        vit_lin = self.VITESSE_NORMAL
                    
                    self.drive(vit_lin, vit_ang)
                    
                # CAS 8 : RIEN -> RECHERCHE ROUTE
                elif self.etat == 0:
                    rospy.loginfo_throttle(1, "PERDU -> RECHERCHE (Rotation Droite)")
                    # Rotation sur place pour chercher la ligne
                    self.drive(0.0, -0.3)                                 
                
                self.pub_vel.publish(self.velocity)
                self.rate.sleep()
                
            except rospy.ROSInterruptException:
                pass

    def drive(self, lin, ang):
        self.velocity.linear.x = lin
        self.velocity.angular.z = ang

if __name__ == '__main__':
    cn = ControleurNavigation()
    cn.run()