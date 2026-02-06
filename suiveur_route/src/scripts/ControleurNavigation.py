#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point

class ControleurNavigation:
    def __init__(self):
        rospy.init_node('ctl_nav_node')
        self.rate = rospy.Rate(10)
       
        self.velocity = Twist()
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub_info = rospy.Subscriber('/navigation/info', Point, self.callback_nav)
       
        # Etats: 0=Rien, 1=Route, 2=Pieton, 3=Fleche
        self.etat = 0
        self.erreur_x = 0.0
       
        # PARAMETRES DE VITESSE
        self.VITESSE_NORMAL = 0.15
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
                    self.drive(self.VITESSE_NORMAL, 0.0)
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
                   
                # CAS 2 : OBSTABLE ROBOT
                elif self.etat == 4:
                    # Ici, pas de sleep. On s'arrete juste pour ce cycle.
                    # Si au prochain cycle le robot est encore la, on reste arrete.
                    # Si il part, l'etat changera et on repartira.
                    rospy.logwarn_throttle(1, "OBSTACLE ROBOT DETECTE -> J'ATTENDS...")
                    self.drive(0.0, 0.0)
                   
                # CAS 3 : FLECHE DETECTE
                elif self.etat == 3:
                    rospy.loginfo_throttle(0.5, "ACTION : VIRAGE DROITE ASSISTE >>>")
                   
                    # On ignore l'erreur X ici pour aller tout droit
                    self.drive(self.VITESSE_LENTE, 0.0)
               
                # CAS 4 : ROUTE -> AVANCE
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
               
                # CAS 5 : RIEN -> ATTENTE
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
