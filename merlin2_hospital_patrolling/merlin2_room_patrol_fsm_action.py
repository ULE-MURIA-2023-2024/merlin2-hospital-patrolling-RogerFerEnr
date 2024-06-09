# TODO: write the patrol FSM action
import rclpy
from typing import List
from .pddl import room_type, room_at, room_patrolled
from merlin2_basic_actions.merlin2_basic_types import wp_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at
from merlin2_fsm_action import Merlin2BasicStates
from merlin2_fsm_action import Merlin2FsmAction

from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin import CbState 
from kant_dto import PddlObjectDto, PddlConditionEffectDto

from geometry_msgs.msg import Twist


class Merlin2RoomPatrolFsmAction(Merlin2FsmAction):

    def __init__(self) -> None:

        self._room = PddlObjectDto(room_type, "room")
        self._wp = PddlObjectDto(wp_type,"wp")
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)


        super().__init__("room_patrol")
    
        tts_state = self.create_state(Merlin2BasicStates.TTS)

        self.add_state(
            "PREPARING_TEXT",
            CbState([SUCCEED],self.prepare_text),
            transitions={
                SUCCEED:"SPEAKING"
            }
        )
        self.add_state(
            "ROTATING",
            CbState([SUCCEED],self.rotate),
            transitions={
                SUCCEED:"PREPARING_TEXT"
            }
        )
#fALTA CREAR EL ESTADO DE GIRAR USANDO EL CMDVEL LE DECIMOS QUE GIRE DANDO UNA VUELTA COMPLETA
#El estado de girar se puede hacer con un callback state
        self.add_state(
            "SPEAKING",
            tts_state
        )

    def prepare_text(self, blackboard: Blackboard)-> str:

        room_name = blackboard.merlin2_action_goal.objects[0][-1]      
        blackboard.text = f"Stretcher room {room_name} patrolled"
        return SUCCEED
    
    def rotate(self, blackboard: Blackboard)-> str:
    	
    	# Publicamos la información en el tópico cmd_vel del robot
        twist = Twist()
        twist.angular.z = 1  # Se puede ajustar la velocidad angular a nuestro gusto
        self.publisher.publish(twist)

        # El robot estará rotando 10 segundos
        self.get_logger().info('El robot ahora está rotando')
        self.create_timer(10.0, self.stop_rotation)

        return SUCCEED

    def stop_rotation(self):
        # Publicamos un mensaje para detener el giro
        twist = Twist()
	twist.angular.z = 0.0
	self.publisher.publish(twist)
	self.get_logger().info('El robot ya ha parado de rotar')


    def create_parameters(self) -> List[PddlObjectDto]:
        return [self._room, self._wp]
    
    def create_conditions(self) -> List[PddlConditionEffectDto]:
        
        cond_1 = PddlConditionEffectDto(
            room_patrolled,
            [self._room],
            PddlConditionEffectDto.AT_START,
            is_negative = False
        )#El robot tiene que estar en la habitación al principio de la acción
        cond_2 = PddlConditionEffectDto(
            robot_at,
            [self._wp],
            PddlConditionEffectDto.AT_START
        ) #La habitación tiene que estar en el mismo lugar que el waypoint al principio de la acción
        cond_3 = PddlConditionEffectDto(
            room_at,
            [self._room,self._wp],
            PddlConditionEffectDto.AT_START    
        )   

        return [cond_1, cond_2, cond_3]     

    def create_effects(self) -> List[PddlConditionEffectDto]:

        effect_1 = PddlConditionEffectDto(
            room_patrolled,
            [self._room]
            time=PddlConditionEffectDto.AT_END 
        )
        
        return [effect_1]

        return super().create_conditions()



def main():
    rclpy.init()
    node = Merlin2RoomPatrolFsmAction()
    node.join_spin()
    rclpy.shutdown()

if __name__ == "main":
    main()
