#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
import time

class ControleurNavigation:
    def __init__(self):
        rospy.init_node('ctl_nav_node')
        self.rate = rospy.Rate(10)
       
        self.velocity = Twist()
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.sub_info = rospy.Subscriber('/navigation/info', Point, self.callback_nav)
       
        # Etats: 0=Rien, 1=Route, 2=Pieton
        self.etat = 0
        self.erreur_x = 0.0
       
        # PARAMETRES
        self.VITESSE_AVANCE = 0.15
        self.GAIN_ROTATION = 0.005 # Augmente car image petite (128px)
       
        # REGLAGE DU TEMPS D'APPROCHE
        # Combien de temps il avance apres avoir detecte le pieton
        self.TEMPS_APPROCHE = 2  # Essaye 1.0 ou 2.0 selon la distance voulue
       
        # Variable pour eviter de detecter le pieton en boucle infini immediatement
        self.cooldown_pieton = False

    def callback_nav(self, msg):
        # msg.z contient le code de l'objet vu
        self.etat = int(msg.z)
        self.erreur_x = msg.x

    def run(self):
        while not rospy.is_shutdown():
            try:
                # CAS 1 : PIETON DETECTE
                if self.etat == 2:
                    # ETAPE A : Approche finale
                    rospy.logwarn("PIETON VU -> APPROCHE FINALE...")
                   
                    # On force le robot a avancer tout droit
                    self.drive(self.VITESSE_AVANCE, 0.0)
                    self.pub_vel.publish(self.velocity)
                   
                    # On le laisse rouler pendant le temps defini
                    rospy.sleep(self.TEMPS_APPROCHE)

                    # ETAPE B : Arret Total
                    rospy.logwarn("ACTION : STOP (Attente 3 sec)")
                    self.drive(0.0, 0.0)
                    self.pub_vel.publish(self.velocity)
                   
                    # Attente bloquante
                    rospy.sleep(3.0)
                   
                    # Reset pour repartir
                    rospy.loginfo("Reprise...")
                    self.etat = 0
               
                # CAS 2 : ROUTE -> AVANCE
                elif self.etat == 1:
                    rospy.loginfo_throttle(1, "ACTION : J'AVANCE (Suivi Route)")
                    vit_ang = -self.erreur_x * self.GAIN_ROTATION
                    self.drive(self.VITESSE_AVANCE, vit_ang)
               
                # CAS 3 : RIEN -> ATTENTE
                else:
                    rospy.loginfo_throttle(1, "ACTION : J'ATTENDS (Recherche Route...)")
                    self.drive(0.0, 0.0)
               
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
