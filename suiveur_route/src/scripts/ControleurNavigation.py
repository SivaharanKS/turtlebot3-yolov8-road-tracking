#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point

class ControleurNavigation:
    def __init__(self):
        rospy.init_node('ctl_nav_node')
        self.rate = rospy.Rate(10) # 10 Hz
       
        self.velocity = Twist()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.sub_info = rospy.Subscriber('/navigation/info', Point, self.callback_nav)
       
        # Variables internes
        self.route_detectee = False
        self.erreur_x = 0.0
       
        # PARAMETRES A REGLER
        self.VITESSE_AVANCE = 0.15      # Vitesse lineaire (m/s)
        self.GAIN_ROTATION = 0.002      # Facteur P pour la rotation
        # Si le robot oscille trop, baisser le GAIN_ROTATION
       
    def callback_nav(self, msg):
        # msg.z contient 1.0 si route detectee, 0.0 sinon
        if msg.z > 0.5:
            self.route_detectee = True
            self.erreur_x = msg.x
        else:
            self.route_detectee = False
            self.erreur_x = 0.0

    def run(self):
        while not rospy.is_shutdown():
            try:
                if self.route_detectee:
                    # Logique de suivi de ligne (Proportionnel)
                    # Si erreur_x est positif (route a droite), on tourne a droite (z negatif)
                    # Note : verifier le sens de rotation de votre robot
                    vit_ang = -self.erreur_x * self.GAIN_ROTATION
                   
                    # On avance et on tourne
                    self.drive(self.VITESSE_AVANCE, vit_ang)
                    rospy.loginfo(f"Suivi Route | Erreur: {self.erreur_x:.1f} | CmdRot: {vit_ang:.3f}")
               
                else:
                    # Pas de route vue -> Arret securite
                    self.drive(0.0, 0.0)
                    rospy.loginfo("Pas de route detectee - Arret")
               
                self.publisher.publish(self.velocity)
                self.rate.sleep()
               
            except rospy.ROSInterruptException:
                pass

    def drive(self, lin, ang):
        self.velocity.linear.x = lin
        self.velocity.angular.z = ang

if __name__ == '__main__':
    cn = ControleurNavigation()
    cn.run()
