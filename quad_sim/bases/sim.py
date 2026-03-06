from abc import ABC
from typing import List
from quad_sim.bases.drone import DroneBase
from quad_sim.bases.configuration import BuildableConfig

class NCopterBase(ABC):
    def __init__(self, agents:List[BuildableConfig], log=None):

        # Check if the agent being passed is top level (droneBase) or not
        self.__checkTopLevel(agents)
        # Construct all the droneBase objects and save them
        self.__entities = {}
        for dr in agents:
            inst = dr.construct()
            self.__entities[inst.iD] = inst

        # Construct the logger
        self.__logger = log

        # Construct the enviornmental models (TBD)
    
    def run(self):
        """
        Advances the simulation by one tick for all entities.

        The caller is responsible for looping over the desired number of ticks.
        Each call steps every entity once and, if a logger is attached, records
        the current state.
        """
        for iD, dr in self.__entities.items():
            dr.step()
        if self.__logger is not None:
            self.__logger.step()

    def close(self):
        if self.__logger is not None:
            self.__logger.finalize()

    def __checkTopLevel(self, agents:List[BuildableConfig]):
        """
        Validates that all agents provided are top-level configurations.
        This method checks if each agent in the provided list has the `__topLevel__` attribute,
        which indicates that the agent is a top-level configuration required for the simulator.

        :param agents: A list of BuildableConfig objects to be validated.
        :raises AttributeError: If any agent in the list does not have the `__topLevel__` attribute.
        """

        # Check if the agent being passed is top level (droneBase) or not
        if not all(hasattr(dr,"__topLevel__") for dr in agents):
            raise AttributeError("Agents must be a list or single topLevel configutation for the simulator to construct necessary elements")
    
    def __checkEntID(self, iD: str) -> bool:
        """
        Checks if the provided entity ID already exists in the simulator.

        This method verifies whether the given entity ID is present in the existing entities.
        If the ID is found, it returns True, indicating a duplicate entity ID.

        :param iD: The entity ID to be checked.
        :return: True if the entity ID already exists, False otherwise.
        """
        if iD in self.__entities:
            # TODO: add a log response
            return True
        return False

    def __addEntities(self,agents: List[BuildableConfig]):
        """
        Adds entities to the simulation by constructing them from the provided configuration.

        This method iterates over the given `agents` configuration, constructs each entity,
        checks for duplicate entity IDs, and adds the constructed entities to the internal
        entity dictionary.

        :param agents: A list of configuration objects containing the specifications for the entities
                       to be added. Each entity is expected to have a `construct` method that
                       returns an instance with a unique `iD` attribute.
        :type agents: List[BuildableConfig]
        """
        for dr in agents:
            inst = dr.construct()
            if not self.__checkEntID(inst.iD):
                self.__entities[inst.iD] = inst


    @property
    def entities(self):
        return self.__entities

    def appendEntity(self, agents: list[BuildableConfig]):
        self.__checkTopLevel(agents)
        self.__addEntities(agents)

    def removeEntity(self, entity: str | DroneBase):
        """
        Removes an entity from the simulation.

        This method allows the removal of an entity from the simulation by providing either
        the entity's ID (as a string) or the entity object itself. If the entity exists in
        the simulation, it is removed from the internal entity dictionary.

        :param entity: The entity to be removed, specified as either a string (entity ID)
                       or a DroneBase object.
        :type entity: str | DroneBase
        :raises TypeError: If the provided entity is neither a string nor a DroneBase object.
        """
        if isinstance(entity, DroneBase):
            iD = entity.iD
        elif isinstance(entity, str):
            iD = entity
        else:
            raise TypeError("Entity must be either a DroneBase object or a string representing the entity ID.")
        
        if self.__checkEntID(iD):
            del self.__entities[iD]
        else:
            raise KeyError(f"Entity with ID '{iD}' does not exist in the simulation.")
            
    # TODO: add the editing functions for the environment
    # def setSimConfig(self, value: SimulatorConfig):
    #     if not isinstance(value, SimulatorConfig):
    #         raise TypeError("New simulation config must be a SimulatorConfig class")
    #     else:
    #         self.__sim = value
