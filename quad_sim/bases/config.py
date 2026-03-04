from abc import ABC, abstractmethod

class BuildableConfig(ABC):
    """
    A high level configuration setup that allows itself and its children config to be built as one of the base model object
    """
    @abstractmethod
    def __init__(self, construct: type):
        """
        Should contain the basics that the configuration can use to construct the base model object
        
        :param construct: The base model class
        :type construct: type
        """
        if not isinstance(construct, type):
            raise TypeError("construct must be a class type")
        if not issubclass(construct, ABC):
            raise TypeError("construct must be a subclass of a baseModel")
        self.__construct = construct
        
    @abstractmethod
    def construct(self) -> ABC:
        """
        Constructs and returns an instance of the base model object.

        :return: An instance of the base model object
        :rtype: ABC
        """
        pass

class DataConfig:
    pass