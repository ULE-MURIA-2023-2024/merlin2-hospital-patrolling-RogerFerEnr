# TODO: write the PDDL domain


from merlin2_basic_actions.merlin2_basic_types import wp_type
from kant_dto import PddlTypeDTO, PddlPredicateDto

room_type = PddlTypeDto("room")

room_patrolled = PddlPredicateDto("room_patrolled",[room_type])

#Hay que decir que waypoint corresponde a cada ubicación
room_at = PddlPredicateDto("room_at",[room_type,wp_type])
#Predicado para decir en que waypoint está cada habitacion
#Con esto ya se ha definido el dominio
