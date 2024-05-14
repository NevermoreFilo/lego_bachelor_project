#!/usr/bin/env python
import time

# Classi del progetto
import ShapeRecognition
import Coordinate

# Librerie standard Python
import sys
import copy
import math

# Ros e Franka
import rospy
import actionlib
import franka_gripper

# MoveIt
import moveit_commander
import moveit_msgs

# Messaggi e server
import geometry_msgs.msg
from control_msgs.msg import GripperCommand, GripperCommandAction
from franka_gripper.msg import MoveAction, GraspAction
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory, Grasp, PlaceLocation, Constraints, OrientationConstraint
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest, GetPlanningScene

from Markers import MarkerDetector
from Vision import Camera


class Robot:

    def __init__(self):
        # Inizializzazione generale
        rospy.init_node("panda_arm_lego_pick_place")  # Inizializzazione nodo ROS
        moveit_commander.roscpp_initialize(sys.argv)  # Inizializzazione moveit_commander
        rospy.wait_for_service("apply_planning_scene")
        try:
            planning_scene_srv = rospy.ServiceProxy(
                "apply_planning_scene", ApplyPlanningScene
            )
        except rospy.ServiceException as e:
            rospy.logerr(
                "Moveit 'apply_planning_scene' service initialization failed: %s" % e
            )
            shutdown_msg = "Shutting down %s node because %s service connection failed." % (
                rospy.get_name(),
                planning_scene_srv.resolved_name,
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        self.get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        self.set_planning_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.planning_scene_interface = moveit_commander.PlanningSceneInterface(ns="/")
        self.move_group.set_planner_id("TRRT")  # Specifico il planner utilizzato
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
        )
        self.start_joint_goal = [0.0, -0.785398163397, 0.0, -2.35619449019, 0.0, 1.57079632679,
                                 0.805398163397]  # Posizione a riposo del robot

        # Dimensione dei lego
        self.lego_width = 0.0314
        self.lego_height = 0.019

        self.open_offset = 0.03  # Offset da aggiungere all'apertura del gripper oltre alla larghezza/lunghezza del lego
        self.closed_offset = 0.005  # Offset da aggiungere alla chiusura del gripper (la dimensionee del lego non e' sufficiente)
        self.eef_height = 0.111  # Offset sull'asse delle z, preso dalla documentazione ufficiale
        self.up_offset = 0.0314  # Offset a cui il braccio si alza dopo aver piazzato il pezzo
        self.initial_up_offset = 0.005

        # Inizializzazione liste di coordinate
        self.starting_long_coordinates = []
        self.starting_long_aux_coords = [[-0.08022024347283004, 0.4345996311647573, -0.017687250096303953],
                                         [-0.077125263174171483, 0.6157033161972792, -0.015843953334895085],
                                         [0.02097427655921029, 0.434620359716768, -0.01736008183053387],
                                         [0.021374736824171483, 0.6157033161972792, -0.015843953334895085],
                                         [0.12239073295454152, 0.4347217843628824, -0.016965818155016486],
                                         [0.4201145557236046, 0.44608137254548264, -0.014633344607714552]]

        self.starting_short_coordinates = []
        self.starting_short_aux_coords = [[0.21788618069731605, 0.3082757604473198, -0.018247824801653288],
                                          [0.22313862587754232, 0.43369528155501386, -0.017351559286898043],
                                          [0.32142728885822247, 0.3071422312992683, -0.017243796812094654],
                                          [0.3226593631314784, 0.43208637933142, -0.015421032728421769]]

        self.place_short_coordinates = []
        self.place_short_aux_coords = [[0.20188832488009007, 0.5854943504460622, 0.01264770456857292]]

        self.place_long_coordinates = []
        self.place_long_aux_coords = [[0.28470540663196856, 0.5983946726759686, 0.012544381106687417],
                                      [0.3158040663196856, 0.5973946726759686, 0.011544381106687417],
                                      [0.39547839242926826, 0.59605657314936886, 0.010568227926752192],
                                      [0.4265770521169853, 0.59505657314936886, 0.010568227926752192]]

        self.mrk_detector = None
        self.realsense = Camera.Camera()

    # Funzione per l'inizializzazione delle liste di coordinate
    def init_coordinates(self):
        for index in range(6):
            self.starting_long_coordinates.append(Coordinate.Coordinate(self.starting_long_aux_coords[index][0],
                                                                        self.starting_long_aux_coords[index][1],
                                                                        self.starting_long_aux_coords[index][2],
                                                                        True))
        for index in range(4):
            self.starting_short_coordinates.append(Coordinate.Coordinate(self.starting_short_aux_coords[index][0],
                                                                         self.starting_short_aux_coords[index][1],
                                                                         self.starting_short_aux_coords[index][2],
                                                                         True))

        for index in range(1):
            self.place_short_coordinates.append(Coordinate.Coordinate(self.place_short_aux_coords[index][0],
                                                                      self.place_short_aux_coords[index][1],
                                                                      self.place_short_aux_coords[index][2],
                                                                      True))
        for index in range(4):
            self.place_long_coordinates.append(Coordinate.Coordinate(self.place_long_aux_coords[index][0],
                                                                     self.place_long_aux_coords[index][1],
                                                                     self.place_long_aux_coords[index][2],
                                                                     True))
        self.mrk_detector = MarkerDetector.MarkerDetector(short_coord_list=self.starting_short_coordinates,
                                           long_coord_list=self.starting_long_coordinates)

    # Funzione che porta il robot ad una posizione "a riposo"
    def go_to_starting_pose(self):
        self.move_group.set_max_velocity_scaling_factor(1)
        self.move_group.set_max_acceleration_scaling_factor(0.06)
        self.move_group.go(self.start_joint_goal, wait=True)

    # Funzione che porta il robot in una posizione "di lavoro": direzionato verso la base e con l'end-effector orientato come i lego
    def go_to_working_pose(self):
        self.move_group.set_max_velocity_scaling_factor(1)
        self.move_group.set_max_acceleration_scaling_factor(0.06)
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = self.start_joint_goal[0] + math.pi / 2
        joint_goal[1] = self.start_joint_goal[1]
        joint_goal[2] = self.start_joint_goal[2]
        joint_goal[3] = self.start_joint_goal[3]
        joint_goal[4] = self.start_joint_goal[4]
        joint_goal[5] = self.start_joint_goal[5]
        joint_goal[6] = self.start_joint_goal[6] + math.pi / 2
        self.move_group.go(joint_goal, wait=True)

    # Funzione per portare il gripper nella posa del primo grip (quella dal lato corto)
    def pre_grasp_rotation(self):
        self.move_group.set_max_velocity_scaling_factor(0.8)
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[6] = joint_goal[6] - math.pi / 2
        self.move_group.go(joint_goal, wait=True)

    # Funzione per portare il gripper nella posa del primo grip (quella dal lato lungo)
    def grasp_rotation(self):
        self.move_group.set_max_velocity_scaling_factor(0.8)
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[6] = joint_goal[6] + math.pi / 2
        self.move_group.go(joint_goal, wait=True)

    # Funzione per far ritirare il braccio verso l'alto dopo aver piazzato / preso
    def upward_retreat(self):
        waypoints = []
        wpose = self.move_group.get_current_pose().pose
        wpose.position.z = wpose.position.z + self.up_offset + self.initial_up_offset
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,
            0.01,
            0.0
        )
        plan = self.move_group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(), plan,
                                                 velocity_scaling_factor=0.8, acceleration_scaling_factor=0.2
                                                 )  # Per rallentare il robot
        self.move_group.execute(plan, wait=True)

    # Funzione per aprire il gripper. Il parametro width dev'essere una stringa che indica il grado di apertura (NARROW o WIDE)
    def open_gripper(self, width):
        if width == "NARROW":
            width_multiplier = 1
        elif width == "WIDE":
            width_multiplier = 2
        else:
            return "Error, incorrect width. Use \"NARROW\" or \"WIDE\""
        # Crea il SimpleActionClient per una MoveAction
        client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)
        client.wait_for_server()
        # Creazione di un goal da mandare al server. L'apertura avviene lentamente per evitare problemi incontrati
        # sperimentalmente
        goal = franka_gripper.msg.MoveGoal(width=width_multiplier * self.lego_width + self.open_offset, speed=0.01)
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    # Funzione per aprire il gripper all'inizio. La differenza con la precedente sta nella velocita'
    def starting_open_gripper(self):

        # Crea il SimpleActionClient per una MoveAction
        client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)
        client.wait_for_server()

        goal = franka_gripper.msg.MoveGoal(width=self.lego_width + self.open_offset, speed=0.04)
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    # Funzione per chiudere il gripper. Il parametro width dev'essere una stringa che indica il grado di apertura (NARROW o WIDE)
    def close_gripper(self, width):
        if width == "NARROW":
            width_multiplier = 1
        elif width == "WIDE":
            width_multiplier = 2
        else:
            return "Error, incorrect width. Use \"NARROW\" or \"WIDE\""
        # Crea il SimpleActionClient per una GraspAction
        client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)
        client.wait_for_server()
        # Creazione di un goal da mandare al server
        goal = franka_gripper.msg.GraspGoal(width=width_multiplier * self.lego_width - self.closed_offset, speed=0.04,
                                            force=10)  # Nota: potrebbe essere sufficiente anche una forza minore
        goal.epsilon.inner = 0.01  # Tolleranza di sfasamento di apertura del gripper rispetto a quella designata
        goal.epsilon.outer = 0.01
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    # Funzione che chiude il gripper. A differenza di close_gripper, la chiusura e' sempre stretta, poiche' serve
    # ad effettuare il "double grasp"
    def secure_gripper(self):
        # Crea il SimpleActionClient per una GraspAction
        client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)
        client.wait_for_server()
        # Creazione di un goal da mandare al server
        goal = franka_gripper.msg.GraspGoal(width=self.lego_width - self.closed_offset, speed=0.05,
                                            force=0.25)
        goal.epsilon.inner = 0.01  # Tolleranza di sfasamento di apertura del gripper rispetto a quella designata
        goal.epsilon.outer = 0.01
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    # Funzione per prendere il pezzo. Il parametro coordinate e' di tipo Coordinate ed indica il punto in cui
    # prendere. Il parametro pieceType dev'essere una stringa che indica il tipo di pezzo da prendere(SMALL o LONG)
    def pick(self, coordinate, pieceType):
        waypoints = []
        self.pre_grasp_rotation()
        self.starting_open_gripper()
        if pieceType == "SMALL":
            width = "NARROW"
        elif pieceType == "LONG":
            width = "WIDE"
        else:
            return "Error, incorrect piece type. Use \"SMALL\" or \"LONG\""
        wpose = self.move_group.get_current_pose().pose
        wpose.position.x = coordinate.x
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y = coordinate.y
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z = coordinate.z + (
                (self.lego_height * 2) / 3) + self.eef_height  # Il grasp avviene a 2/3 circa dell'altezza del lego
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,
            0.01,
            0.0
        )
        plan = self.move_group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(), plan,
                                                 velocity_scaling_factor=0.9, acceleration_scaling_factor=0.2
                                                 )  # Per rallentare il robot

        self.move_group.execute(plan, wait=True)
        # self.close_gripper("NARROW")
        self.secure_gripper()  # Effettuo il "double grip" per raddrizzare il pezzo
        rospy.sleep(0.5)
        self.open_gripper(width)
        rospy.sleep(0.5)
        self.grasp_rotation()
        wpose.position.z = coordinate.z + (
                (self.lego_height * 2) / 3) + self.eef_height  # Il grasp avviene a 2/3 circa dell'altezza del lego
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,
            0.01,
            0.0
        )
        plan = self.move_group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(), plan,
                                                 velocity_scaling_factor=0.8, acceleration_scaling_factor=0.15
                                                 )  # Per rallentare il robot
        self.move_group.execute(plan, wait=True)
        self.close_gripper(width)
        self.upward_retreat()

    # Funzione per piazzare il pezzo. Il parametro coordinate e' di tipo Coordinate ed indica il punto in cui
    # piazzare. Il parametro pieceType dev'essere una stringa che indica il tipo di pezzo da piazzare(SMALL o LONG)
    # Il parametro z_offset indica di quanto bisogna alzarsi rispetto alla coordinata originale
    def place(self, coordinate, piece_type, z_offset):
        if piece_type == "SMALL":
            width = "NARROW"
        elif piece_type == "LONG":
            width = "WIDE"
        else:
            return "Error, incorrect piece type. Use \"SMALL\" or \"LONG\""

        waypoints = []
        wpose = self.move_group.get_current_pose().pose
        wpose.position.x = coordinate.x
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y = coordinate.y - 0.001
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z = coordinate.z + self.eef_height - 0.0075 + z_offset  # Scendo alla coordinata + poco meno della dimensione dell'end effector
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,
            0.01,
            0.0
        )
        plan = self.move_group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(), plan,
                                                 velocity_scaling_factor=0.8,
                                                 acceleration_scaling_factor=0.1)  # Per rallentare il robot
        self.move_group.execute(plan, wait=True)
        self.open_gripper(width)
        self.upward_retreat()

    # Funzione che itera le coordinate relative ai pezzi lunghi fino a trovarne uno disponibile. Se non lo trova, termina il programma
    def find_suitable_long_piece(self):
        self.go_to_working_pose()
        time.sleep(5.0)
        self.realsense.get_frame()
        self.mrk_detector.detect()
        i = 0
        while not self.starting_long_coordinates[i].active:
            i = i + 1
            if i == 6:
                print("Non e' stato rilevato un pezzo lungo conforme")
                sys.exit()
        return i

    # Funzione che itera le coordinate relative ai pezzi corti fino a trovarne uno disponibile. Se non lo trova, termina il programma
    def find_suitable_short_piece(self):
        self.go_to_working_pose()
        time.sleep(5.0)
        self.realsense.get_frame()
        self.mrk_detector.detect()
        i = 0
        while not self.starting_short_coordinates[i].active:
            i = i + 1
            if i == 4:
                print("Non e' stato rilevato un pezzo corto conforme")
                sys.exit()
        return i

    # Funzioni che controllano se i pezzi presenti sono sufficienti per comporre le varie figure
    def initial_tower_check(self):
        time.sleep(5.0)
        self.realsense.get_frame()
        self.mrk_detector.detect()
        j = 0
        for i in range(0, 4):
            if self.starting_short_coordinates[i].active:
                j = j + 1

        if j < 4:
            print("Pezzi corti insufficienti")
            sys.exit()

    def initial_rectangle_check(self):
        time.sleep(5.0)
        self.realsense.get_frame()
        self.mrk_detector.detect()
        j = 0
        for i in range(0, 6):
            if self.starting_long_coordinates[i].active:
                j = j + 1
        if j < 2:
            print("Pezzi lunghi insufficienti")
            sys.exit()

    def initial_square_check(self):
        time.sleep(5.0)
        self.realsense.get_frame()
        self.mrk_detector.detect()
        j = 0
        for i in range(0, 6):
            if self.starting_long_coordinates[i].active:
                print j
                j = j + 1
        if j < 4:
            print("Pezzi lunghi insufficienti")
            sys.exit()
            
    # Seguono le funzioni per costruire le figure. Regola generale: prima e dopo iniziare la costruzione, il braccio si
    # porta nella sua "working position"

    # Funzione per far costruire la torre al braccio robotico
    def build_tower(self):
        #self.go_to_working_pose()
        self.initial_tower_check()

        # Piazzo i 4 blocchi, ogni volta alzando l'altezza di cui si ritira il braccio
        i = self.find_suitable_short_piece()

        self.pick(self.starting_short_coordinates[i], "SMALL")
        self.place(self.place_short_coordinates[0], "SMALL", 0)
        if self.up_offset == 0.0314:
            self.up_offset += self.lego_height

        i = self.find_suitable_short_piece()

        self.pick(self.starting_short_coordinates[i], "SMALL")
        self.place(self.place_short_coordinates[0], "SMALL", self.lego_height)
        self.up_offset += self.lego_height

        i = self.find_suitable_short_piece()

        self.pick(self.starting_short_coordinates[i], "SMALL")
        self.place(self.place_short_coordinates[0], "SMALL", 2 * self.lego_height)
        self.up_offset += self.lego_height

        i = self.find_suitable_short_piece()

        self.pick(self.starting_short_coordinates[i], "SMALL")
        self.place(self.place_short_coordinates[0], "SMALL", 3 * self.lego_height)
        self.up_offset += self.lego_height

        self.go_to_working_pose()

    # Funzione per far costruire il rettangolo al braccio robotico
    def build_rectangle(self):
        #self.go_to_working_pose()
        self.initial_rectangle_check()

        i = self.find_suitable_long_piece()

        # Piazzo i 2 blocchi. Per ragioni sperimentali, risulta piu' comodo piazzare prima il pezzo piu' in fondo
        self.pick(self.starting_long_coordinates[i], "LONG")
        self.place(self.place_long_coordinates[1], "LONG", 0)

        i = self.find_suitable_long_piece()

        self.pick(self.starting_long_coordinates[i], "LONG")
        self.place(self.place_long_coordinates[0], "LONG", 0)

        self.go_to_working_pose()

    # Funzione per far costruire il rettangolo al braccio robotico
    def build_square(self):
        #self.go_to_working_pose()
        self.initial_square_check()

        i = self.find_suitable_long_piece()

        # Piazzo i 4 blocchi. Per ragioni sperimentali, risulta piu' comodo piazzare prima il pezzo piu' in fondo
        self.pick(self.starting_long_coordinates[i], "LONG")
        self.place(self.place_long_coordinates[3], "LONG", 0)

        i = self.find_suitable_long_piece()

        self.pick(self.starting_long_coordinates[i], "LONG")
        self.place(self.place_long_coordinates[2], "LONG", 0)

        if self.up_offset == 0.0314:  # Alzo l'offset solo se non era gia' stato alzato
            self.up_offset += self.lego_height

        i = self.find_suitable_long_piece()

        self.pick(self.starting_long_coordinates[4], "LONG")
        self.place(self.place_long_coordinates[3], "LONG", self.lego_height)

        i = self.find_suitable_long_piece()

        self.pick(self.starting_long_coordinates[5], "LONG")
        self.place(self.place_long_coordinates[2], "LONG", self.lego_height)

        self.go_to_working_pose()
