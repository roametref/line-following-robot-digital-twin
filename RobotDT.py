import math
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt
import random

#PORTS
@dataclass
class SensorReadingPort:
    reading : float
@dataclass
class SensorPositionPort:
    x : float
    y : float
@dataclass
class ServoCommandPort:
    servoVal : float
@dataclass
class RobotPositionPort:
    x : float
    y : float
    theta : float
@dataclass
class EnergyPort:
    energy : float
@dataclass
class AngularVelocityPort:
    angular_vel : float
@dataclass
class LinearVelocityPort:
    linear_vel : float


#COMPOSANTS
#Controleur
class DigitalController :
    #Constantes de valeurs de vitesse
    FORWARD_SPEED = 0.4
    FORWARD_ROTATE = 0.5
    BACKWARD_ROTATE = 0.1
    #Constante pour le seuil
    THRESHOLD_LINE = 150.0
    def __init__(self,lfLeftVal :SensorReadingPort, lfRightVal : SensorReadingPort, servoRightOut : ServoCommandPort, servoLeftOut : ServoCommandPort) :
        #IN
        self.lfLeftVal = lfLeftVal
        self.lfRightVal = lfRightVal
        #OUT
        self.servoRightOut = servoRightOut
        self.servoLeftOut = servoLeftOut 

        self.on_line = False
        self.last_seen = "left"
        self.searching = True  # ← phase de recherche initiale

    def line_following_behaviour(self):
        left_black  = self.lfLeftVal.reading  < self.THRESHOLD_LINE
        right_black = self.lfRightVal.reading < self.THRESHOLD_LINE

        # Phase recherche initiale : tourner jusqu'à trouver la ligne
        if self.searching:
            if left_black or right_black:
                self.searching = False
                self.on_line = True
            else:
                # Avancer
                self.servoRightOut.servoVal =  self.FORWARD_ROTATE
                self.servoLeftOut.servoVal  = self.BACKWARD_ROTATE
                return
        # Avancer tout droit
        if left_black and right_black:
            # léger biais selon dernier côté vu
            if self.last_seen == "left":
                self.servoRightOut.servoVal = self.FORWARD_SPEED
                self.servoLeftOut.servoVal  = self.FORWARD_SPEED 
            else:
                self.servoRightOut.servoVal = self.FORWARD_SPEED 
                self.servoLeftOut.servoVal  = self.FORWARD_SPEED
        # Tourner à gauche
        elif left_black and not right_black:
            self.last_seen = "left"
            self.servoRightOut.servoVal = self.FORWARD_ROTATE
            self.servoLeftOut.servoVal  = self.BACKWARD_ROTATE

        # Tourner à droite
        elif right_black and not left_black:
            self.last_seen = "right"
            self.servoRightOut.servoVal = self.BACKWARD_ROTATE
            self.servoLeftOut.servoVal  = self.FORWARD_ROTATE

        # Ligne perdue : Recherche
        else:
            if self.last_seen == "left":
                self.servoRightOut.servoVal = self.FORWARD_ROTATE
                self.servoLeftOut.servoVal  = -self.FORWARD_ROTATE
            else:
                self.servoRightOut.servoVal = -self.FORWARD_ROTATE
                self.servoLeftOut.servoVal  = self.FORWARD_ROTATE

        # Contraintes des valeurs servo : [-1,1]
        assert -1.0 <= self.servoRightOut.servoVal <= 1.0, (
            f"ServoRightBounds violated"
        )
        assert -1.0 <= self.servoLeftOut.servoVal <= 1.0, (
            f"ServoLeftBounds violated"
        )

        """self.last_seen_right = False
        self.last_seen_left = False"""
    
    """def line_following_behaviour(self) :
        if self.lfRightVal.reading > self.THRESHOLD_LINE and self.lfLeftVal.reading > self.THRESHOLD_LINE :
            #Avancer tout droit
            self.servoRightOut.servoVal = self.FORWARD_SPEED
            self.servoLeftOut.servoVal = self.FORWARD_SPEED
        elif self.lfRightVal.reading > self.THRESHOLD_LINE and self.lfLeftVal.reading < self.THRESHOLD_LINE:
            #Tourner à gauche
            self.servoRightOut.servoVal = self.FORWARD_ROTATE
            self.servoLeftOut.servoVal = self.BACKWARD_ROTATE
            self.last_seen_left = True
        elif self.lfRightVal.reading < self.THRESHOLD_LINE and self.lfLeftVal.reading > self.THRESHOLD_LINE:
            #Tourner à droite
            self.servoRightOut.servoVal = self.BACKWARD_ROTATE
            self.servoLeftOut.servoVal = self.FORWARD_ROTATE
            self.last_seen_right = True
        else : #Les deux capteurs voient du noir
            if self.last_seen_right :
                #Tourner à droite
                self.servoRightOut.servoVal = self.BACKWARD_ROTATE
                self.servoLeftOut.servoVal = self.FORWARD_ROTATE
            else :
                if self.last_seen_left :
                    #Tourner à gauche
                    self.servoRightOut.servoVal = self.FORWARD_ROTATE
                    self.servoLeftOut.servoVal = self.BACKWARD_ROTATE

        #Contraintes des valeurs servo : [-1,1]
        assert -1.0 <= self.servoRightOut.servoVal <= 1.0, (
            f"ServoRightBounds violated"
        )
        assert -1.0 <= self.servoLeftOut.servoVal <= 1.0, (
            f"ServoLeftBounds violated"
        )"""

#Capteurs IR
class DigitalIRSensor :
    #Constantes 
    LF_POSITION_X = 0.065 #Fixe pour les deux capteurs
    def __init__(self,lf_position_y, robotPos : RobotPositionPort, sensorPos : SensorPositionPort, opticalReflection : SensorReadingPort, sensorVal : SensorReadingPort) :
        self.lf_position_x = self.LF_POSITION_X
        self.lf_position_y = lf_position_y
        
        self.rotated_x = 0.0
        self.rotated_y = 0.0
        self.new_lf_pos_x = 0.0
        self.new_lf_pos_y = 0.0

        #IN
        self.robotPos = robotPos
        self.opticalReflection = opticalReflection
        #OUT
        self.sensorPos = sensorPos
        self.sensorVal = sensorVal

    def update_sensor_position(self) :
        self.rotated_x = self.lf_position_x * math.cos(self.robotPos.theta) - self.lf_position_y * math.sin(self.robotPos.theta)
        self.rotated_y = self.lf_position_x * math.sin(self.robotPos.theta) + self.lf_position_y * math.cos(self.robotPos.theta)
        self.new_lf_pos_x  = self.robotPos.x + self.rotated_x
        self.new_lf_pos_y  = self.robotPos.y + self.rotated_y
        #Export
        self.sensorPos.x = self.new_lf_pos_x
        self.sensorPos.y = self.new_lf_pos_y

    def update_sensor_reading(self) :
        #Contrainte
        assert 0.0 <= self.opticalReflection.reading <= 255.0, (
            f"OpticalReflectionBounds violated"
        )
        self.sensorVal.reading = self.opticalReflection.reading

#Moteur
class DigitalMotor :
    #Constantes
    K = 5.18 # Gain : rad/s
    MOTOR_POWER_MAX = 10.0 #W
    def __init__(self,servoIn : ServoCommandPort,toWheel : AngularVelocityPort, toEncoder : AngularVelocityPort, servoJ : EnergyPort) :
        self.energy_used = 0.0
        #IN
        self.servoIn = servoIn
        #OUT
        self.toWheel = toWheel
        self.toEncoder = toEncoder
        self.servoJ = servoJ

    def update_angular_velocity(self) :
        angular_velocity = self.K * self.servoIn.servoVal
        #Export
        self.toWheel.angular_vel = angular_velocity
        self.toEncoder.angular_vel = angular_velocity
    
    def update_energy(self, dt : float) :
        #Puissance proportionnelle à la commande servo
        proportionnal_power = self.MOTOR_POWER_MAX * abs(self.servoIn.servoVal)
        self.energy_used += proportionnal_power * dt
        #Export
        self.servoJ.energy = self.energy_used

#Roue
class DigitalWheel :
    WHEEL_RADIUS = 0.03325 #m
    def __init__(self,angularVelIn : AngularVelocityPort, linearVelOut : LinearVelocityPort) :
        #IN
        self.angularVelIn = angularVelIn
        #OUT
        self.linearVelOut = linearVelOut

    def update_linear_velocity(self) :
        #Calcul & Export
        self.linearVelOut.linear_vel = self.WHEEL_RADIUS * self.angularVelIn.angular_vel

#Codeur
class DigitalEncoder :
    RESOLUTION = 44.0 # ticks/tour
    def __init__(self,angularVelIn : AngularVelocityPort) :
        #IN
        self.angularVelIn = angularVelIn

        self.motor_angle = 0.0 #rad
        self.encoder_continous = 0.0
    
    def update_encoder(self, dt : float) :
        #Calcul angle moteur
        self.motor_angle += self.angularVelIn.angular_vel * dt
        self.encoder_continous = self.motor_angle * self.RESOLUTION / (2.0 * math.pi)

#Batterie
class DigitalBattery :
    BATTERY_INITIAL_ENERGY = 2500.0
    TOTAL_SENSORS_POWER = 1.0
    CONTROLLLER_POWER = 7.0
    def __init__(self, servoLeftJ : EnergyPort, servoRightJ : EnergyPort, totalEnergyUsed : EnergyPort) :
        self.remaing_energy = self.BATTERY_INITIAL_ENERGY #Initialement
        self.used_energy = 0.0
        self.sensor_energy_used = 0.0
        self.controller_energy_used = 0.0

        #IN
        self.servoLeftJ = servoLeftJ
        self.servoRightJ = servoRightJ
        #OUT
        self.totalEnergyUsed = totalEnergyUsed

    def update_energy(self, dt : float) :
        #Calcul energie utilisée par les capteurs
        self.sensor_energy_used += self.TOTAL_SENSORS_POWER * dt
        #Calcul energie utilisée par le controleur
        self.controller_energy_used += self.CONTROLLLER_POWER * dt
        #Calcul énergie totale utilisée
        self.used_energy = self.sensor_energy_used + self.controller_energy_used + self.servoLeftJ.energy + self.servoRightJ.energy
        self.remaing_energy = self.BATTERY_INITIAL_ENERGY - self.used_energy
        #Export
        self.totalEnergyUsed.energy = self.used_energy
        #Contrainte
        assert self.remaing_energy >= 0.0, (
            f"EnergyBounds violated : Batterie vide"
        )


#Chassis
class DigitalChassis :
    DEMI_ENTRAXE = 0.049 #m
    def __init__(self, x0, y0, theta0, linearVelLeftIn : LinearVelocityPort, linearVelRightIn : LinearVelocityPort, robotPos : RobotPositionPort) :
        self.linear_vel = 0.0 #Vitesse lineaire du robot
        self.omega = 0.0 #Vitesse angulaire du robot

        #IN
        self.linearVelLeft = linearVelLeftIn
        self.linearVelRight = linearVelRightIn
        #OUT
        self.robotPos = robotPos

        #Position initiale du robot
        self.robotPos.x = x0
        self.robotPos.y = y0
        self.robotPos.theta = theta0

    def update_robot_position(self, dt : float) :
        vL = self.linearVelLeft.linear_vel
        vR = self.linearVelRight.linear_vel
        #Contrainte calcul vitesses
        self.linear_vel = (vR + vL) / 2.0
        self.omega = (vR - vL) / (2.0 * self.DEMI_ENTRAXE)
        #Calcul & Exportde la nouvelle position du robot
        self.robotPos.theta += self.omega * dt
        self.robotPos.x += self.linear_vel * math.cos(self.robotPos.theta) * dt
        self.robotPos.y += self.linear_vel * math.sin(self.robotPos.theta) * dt

#BodyBlock
class DigitalBodyBlock :
    def __init__(self, x0, y0, theta0, servoRightInput : ServoCommandPort, servoLeftInput : ServoCommandPort, robotPos : RobotPositionPort, totalEnergyUsed : EnergyPort) :
        #IN (externes)
        self.servoRightInput = servoRightInput
        self.servoLeftInput = servoLeftInput
        #OUT (externes)
        self.robotPos = robotPos
        self.totalEnergyUsed = totalEnergyUsed

        #Ports internes (connexions entre les composants)
        #Moteur gauche
        motorL_toWheel = AngularVelocityPort(0.0)
        motorL_toEncoder = AngularVelocityPort(0.0)
        motorL_servoJ = EnergyPort(0.0)
        #Moteur droit
        motorR_toWheel = AngularVelocityPort(0.0)
        motorR_toEncoder = AngularVelocityPort(0.0)
        motorR_servoJ = EnergyPort(0.0)
        #Roue gauche
        wheelL_linearVelOut = LinearVelocityPort(0.0)
        #Roue droite
        wheelR_linearVelOut = LinearVelocityPort(0.0)

        #Composants
        self.motorL = DigitalMotor(servoIn = servoLeftInput, toWheel = motorL_toWheel, toEncoder = motorL_toEncoder, servoJ = motorL_servoJ)
        self.motorR = DigitalMotor(servoIn = servoRightInput, toWheel = motorR_toWheel, toEncoder = motorR_toEncoder, servoJ = motorR_servoJ)
        self.wheelL = DigitalWheel(angularVelIn = motorL_toWheel, linearVelOut = wheelL_linearVelOut)
        self.wheelR = DigitalWheel(angularVelIn = motorR_toWheel, linearVelOut = wheelR_linearVelOut)
        self.encoderL = DigitalEncoder(angularVelIn = motorL_toEncoder)
        self.encoderR = DigitalEncoder(angularVelIn = motorR_toEncoder)
        self.body = DigitalChassis(x0 = x0, y0 = y0, theta0 = theta0, linearVelLeftIn = wheelL_linearVelOut, linearVelRightIn = wheelR_linearVelOut, robotPos = robotPos)
        self.battery = DigitalBattery(servoLeftJ = motorL_servoJ, servoRightJ = motorR_servoJ, totalEnergyUsed = totalEnergyUsed)

    def update(self, dt : float) :
        #Suivant l'ordre du flux de données
        #Moteurs
        #Calcul vitesse angulaire à partir des commandes servo
        self.motorL.update_angular_velocity()
        self.motorR.update_angular_velocity()
        #Calcul énergie consommée
        self.motorL.update_energy(dt)
        self.motorR.update_energy(dt)
        #Roues
        #Conversion vitesse angulaire en vitesse linéaire
        self.wheelL.update_linear_velocity()
        self.wheelR.update_linear_velocity()
        #Encodeurs 
        #Calculs
        self.encoderL.update_encoder(dt)
        self.encoderR.update_encoder(dt)
        #Batterie
        self.battery.update_energy(dt)
        #Chassis
        #MAJ position
        self.body.update_robot_position(dt)

#Line
class Line :
    LINE_WIDTH = 0.02 #m
    def __init__(self, robotPos : RobotPositionPort, sensorLPos : SensorPositionPort, sensorRPos : SensorPositionPort, opticalReflectionLeft : SensorReadingPort, opticalReflectionRight : SensorReadingPort, line_function) :
        #IN
        self.robotPos = robotPos
        self.sensorLPos = sensorLPos
        self.sensorRPos = sensorRPos
        #OUT
        self.opticalReflectionLeft = opticalReflectionLeft
        self.opticalReflectionRight = opticalReflectionRight

        # Historique pour matplotlib
        self.history_x = []
        self.history_y = []
        self.history_sensorL = []
        self.history_sensorR = []

        self.line_function = line_function if line_function is not None else (lambda x: x)  # Par défaut, y=x
    
    def map_lookup (self, x, y) :
        distance = self.line_function(x, y)
        if distance < self.LINE_WIDTH:
            return 0.0 # Noir
        else :
            return 255.0 #Blanc
    
    def update(self) :
        self.opticalReflectionLeft.reading = self.map_lookup(self.sensorLPos.x, self.sensorLPos.y)
        self.opticalReflectionRight.reading = self.map_lookup(self.sensorRPos.x, self.sensorRPos.y)
        
        #MAJ Historique pour l'affichage
        self.history_x.append(self.robotPos.x)
        self.history_y.append(self.robotPos.y)
        self.history_sensorL.append((self.sensorLPos.x,self.sensorLPos.y))
        self.history_sensorR.append((self.sensorRPos.x,self.sensorRPos.y))
        
#Environnement
class Environment :
    def __init__(self, robotPos: RobotPositionPort, sensorLPos: SensorPositionPort, sensorRPos: SensorPositionPort, opticalReflectionLeft: SensorReadingPort, opticalReflectionRight: SensorReadingPort, line_function) :
        #IN (externes)
        self.robotPos = robotPos
        self.sensorLPos = sensorLPos
        self.sensorRPos = sensorRPos
        #OUT (externes)
        self.opticalReflectionLeft = opticalReflectionLeft
        self.opticalReflectionRight = opticalReflectionRight

        self.line = Line(robotPos = self.robotPos, sensorLPos = self.sensorLPos, sensorRPos = self.sensorRPos, opticalReflectionLeft = self.opticalReflectionLeft, opticalReflectionRight = self.opticalReflectionRight, line_function = line_function)

    def update(self) :
        self.line.update()

#Robot 
class DigitalRobot :
    def __init__(self, x0, y0, theta0, opticalReflectionLeft: SensorReadingPort, opticalReflectionRight: SensorReadingPort, robotPos: RobotPositionPort, sensorLPos: SensorPositionPort, sensorRPos: SensorPositionPort, totalEnergyUsed: EnergyPort) :
        #IN (externes)
        self.opticalReflectionLeft = opticalReflectionLeft
        self.opticalReflectionRight = opticalReflectionRight
        #OUT (externes)
        self.robotPos = robotPos
        self.sensorLPos = sensorLPos
        self.sensorRPos = sensorRPos
        self.totalEnergyUsed = totalEnergyUsed

        #Ports internes (connexions entre les composants)
        #Capteurs
        sensorValL = SensorReadingPort(255.0)
        sensorValR = SensorReadingPort(255.0)
        #Controleur
        servoLeft = ServoCommandPort(0.0)
        servoRight = ServoCommandPort(0.0)
        #Composants
        self.bodyBlock = DigitalBodyBlock(x0=x0, y0=y0, theta0=theta0, servoRightInput = servoRight, servoLeftInput = servoLeft, robotPos = robotPos, totalEnergyUsed = totalEnergyUsed)
        self.sensorL = DigitalIRSensor(lf_position_y = 0.01, robotPos = robotPos, sensorPos = sensorLPos, opticalReflection = opticalReflectionLeft, sensorVal = sensorValL)
        self.sensorR = DigitalIRSensor(lf_position_y = -0.01, robotPos = robotPos, sensorPos = sensorRPos, opticalReflection = opticalReflectionRight, sensorVal = sensorValR)
        self.controller = DigitalController(lfLeftVal = sensorValL, lfRightVal = sensorValR, servoRightOut = servoRight, servoLeftOut = servoLeft)

class LFRobotSystem :
    def __init__(self, x0, y0, theta0, line_function) :
        #Ports
        self.robotPos = RobotPositionPort(x0, y0, theta0)
        self.sensorLPos = SensorPositionPort(0.0, 0.0)
        self.sensorRPos = SensorPositionPort(0.0, 0.0)
        self.opticalReflectionLeft = SensorReadingPort(255.0)
        self.opticalReflectionRight = SensorReadingPort(255.0)
        self.totalEnergyUsed = EnergyPort(0.0)
        #Composants
        self.robot = DigitalRobot(x0 = x0, y0 = y0, theta0 = theta0, opticalReflectionLeft = self.opticalReflectionLeft, opticalReflectionRight = self.opticalReflectionRight, robotPos = self.robotPos, sensorLPos = self.sensorLPos, sensorRPos = self.sensorRPos, totalEnergyUsed = self.totalEnergyUsed)
        self.environment = Environment(robotPos = self.robotPos, sensorLPos = self.sensorLPos, sensorRPos = self.sensorRPos, opticalReflectionLeft = self.opticalReflectionLeft, opticalReflectionRight = self.opticalReflectionRight, line_function = line_function)

    def update(self, dt : float) :
        #Suivant l'ordre du flux de données
        self.robot.sensorL.update_sensor_position()
        self.robot.sensorR.update_sensor_position()
        self.environment.update()
        self.robot.sensorL.update_sensor_reading()
        self.robot.sensorR.update_sensor_reading()
        self.robot.controller.line_following_behaviour()
        self.robot.bodyBlock.update(dt)

    def run(self, n_steps) :
        dt = 0.01 #s
        for i in range(n_steps) :
            self.update(dt)
            if i % 100 == 0:  # afficher tous les 100 pas
                print(f"[{i}] pos=({self.robotPos.x:.3f}, {self.robotPos.y:.3f}, {math.degrees(self.robotPos.theta):.1f}°)"
                  f" | sensorL={self.robot.sensorL.sensorVal.reading:.0f}"
                  f" | sensorR={self.robot.sensorR.sensorVal.reading:.0f}"
                  f" | servoL={self.robot.controller.servoLeftOut.servoVal:.2f}"
                  f" | servoR={self.robot.controller.servoRightOut.servoVal:.2f}")
        return self.environment.line
    

#MAIN
if __name__ == "__main__":
    #Fonctions de ligne possibles
    def cercle(x, y):
        r, cx, cy = 0.5, 0.0, 0.5
        return abs(math.sqrt((x-cx)**2 + (y-cy)**2) - r)

    def ligne_droite(x, y):
        return abs(y)

    def sinus(x, y):
        return abs(y - math.sin(x))

    def cosinus(x, y):
        return abs(y - math.cos(x))

    def make_ovale(seed, n_bumps=12, deform=0.08):
        rng = random.Random(seed)
        angles = [i * 2 * math.pi / n_bumps for i in range(n_bumps)]
        radii  = [0.4 + rng.uniform(-deform, deform) for _ in range(n_bumps)]
        angles.append(angles[0] + 2 * math.pi)
        radii.append(radii[0])

        def ovale(x, y):
            dx, dy = x, y
            phi = math.atan2(dy, dx) % (2 * math.pi)
            for i in range(len(angles) - 1):
                if angles[i] <= phi < angles[i + 1]:
                    t = (phi - angles[i]) / (angles[i + 1] - angles[i])
                    t_smooth = (1 - math.cos(t * math.pi)) / 2
                    r_ref = radii[i] * (1 - t_smooth) + radii[i + 1] * t_smooth
                    break
            else:
                r_ref = radii[0]
            r_point = math.sqrt(dx**2 + dy**2)
            return abs(r_point - r_ref)

        ovale.angles = angles
        ovale.radii  = radii
        return ovale

    # Ovale fixe (seed fixe)
    ovale_deforme = make_ovale(seed=7)

    # Ovale aléatoire (seed différent à chaque lancement)
    seed_aleatoire = random.randint(0, 10000)
    ovale_random   = make_ovale(seed=seed_aleatoire)
    print(f"Ovale aléatoire généré avec seed={seed_aleatoire}")

    #Options user
    print("Choisissez une fonction de ligne :")
    print("1. Sinus")
    print("2. Cosinus")
    print("3. Cercle")
    print("4. Ligne droite")
    print("5. Ovale déformé (fixe)")
    print("6. Ovale aléatoire")

    choice = input("Entrez votre choix (1-6) : ")

    if choice == "1":
        system = LFRobotSystem(x0=0.0, y0=0.0, theta0=0.0, line_function=sinus)
    elif choice == "2":
        system = LFRobotSystem(x0=0.0, y0=1.0, theta0=0.0, line_function=cosinus)
    elif choice == "3":
        system = LFRobotSystem(x0=0.0, y0=0.0, theta0=0.0, line_function=cercle)
    elif choice == "4":
        system = LFRobotSystem(x0=0.0, y0=0.0, theta0=0.0, line_function=ligne_droite)
    elif choice == "5":
        system = LFRobotSystem(x0=0.4, y0=0.0, theta0=math.pi/2, line_function=ovale_deforme)
    elif choice == "6":
        system = LFRobotSystem(x0=0.4, y0=0.0, theta0=math.pi/2, line_function=ovale_random)
    else:
        print("Choix invalide. Utilisation du cercle par défaut.")
        system = LFRobotSystem(x0=0.0, y0=0.0, theta0=0.0, line_function=cercle)

    line = system.run(n_steps=4000)

    #Affichage
    fig, ax = plt.subplots(figsize=(12, 5))

    if line.line_function == sinus:
        x_ref = np.linspace(-1, 10, 1000)
        ax.plot(x_ref, np.sin(x_ref), 'k-', linewidth=1.5, label='sin(x)')

    elif line.line_function == cosinus:
        x_ref = np.linspace(-1, 10, 1000)
        ax.plot(x_ref, np.cos(x_ref), 'k-', linewidth=1.5, label='cos(x)')

    elif line.line_function == cercle:
        R, cx, cy = 0.5, 0.0, 0.5
        theta_ref = np.linspace(0, 2*math.pi, 500)
        ax.plot(cx + R*np.cos(theta_ref), cy + R*np.sin(theta_ref), 'k-', linewidth=1.5, label='Cercle')

    elif line.line_function in (ovale_deforme, ovale_random):
        # Même logique d'affichage pour les deux ovales
        ovale_actif = line.line_function
        label       = 'Ovale déformé (fixe)' if ovale_actif == ovale_deforme else f'Ovale aléatoire (seed={seed_aleatoire})'
        angles_ref  = ovale_actif.angles
        radii_ref   = ovale_actif.radii
        theta_ref   = np.linspace(0, 2*math.pi, 1000)
        ref_x, ref_y = [], []
        for t in theta_ref:
            phi = t % (2 * math.pi)
            r_ref = radii_ref[0]
            for i in range(len(angles_ref) - 1):
                if angles_ref[i] <= phi < angles_ref[i + 1]:
                    interp        = (phi - angles_ref[i]) / (angles_ref[i + 1] - angles_ref[i])
                    interp_smooth = (1 - math.cos(interp * math.pi)) / 2
                    r_ref         = radii_ref[i] * (1 - interp_smooth) + radii_ref[i + 1] * interp_smooth
                    break
            ref_x.append(r_ref * math.cos(t))
            ref_y.append(r_ref * math.sin(t))
        ax.plot(ref_x, ref_y, 'k-', linewidth=1.5, label=label)

    else:  # ligne_droite
        x_ref = np.linspace(-1, 5, 100)
        ax.plot(x_ref, np.zeros_like(x_ref), 'k-', linewidth=1.5, label='y=0')

    ax.plot(line.history_x, line.history_y, 'b-', linewidth=2, markersize=4, label='Trajectoire robot')
    ax.plot(line.history_x[0], line.history_y[0], 'go', markersize=8, label='Départ')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.show()