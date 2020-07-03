from abc import ABC, abstractmethod 
  
class RobotClient(ABC): 
  
    ##############################
    # Abstract Retrieval Methods #
    ##############################
    def get_odom(self): 
        pass
    
    def get_vision(self): 
        pass

    def get_laserscan(self):
        pass

    def get_robot_status(self):
        pass

    ##############################
    # Abstract Commander Methods #
    ##############################

    def set_waypoint(self):
        pass

    def set_estop_behavior(self):
        pass

    def start_robot(self):
        pass

    def stop_robot(self):
        pass

    def connect(self):
        pass

