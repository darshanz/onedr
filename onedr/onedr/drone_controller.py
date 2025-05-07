import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping 
# TODO NOT sure why pymavlink has issue iwth MutableMapping; just used older method for now.. check later
from pymavlink import mavutil
 

class DroneController:
    def __init__(self, logger):
        """Initialize the DroneController with default values."""
        self.connection = None
        self.connected = False
        self.target_system = 1  # Default system ID
        self.target_component = 1  # Default component ID
        self.logger = logger
    
    def connect(self, connection_string):
        try:
            
            self.connection = mavutil.mavlink_connection(connection_string)
            self.logger.info("Waiting for heartbeat from the drone...")
            self.connection.wait_heartbeat()
            self.logger.info("Connected to drone! System ID: {self.connection.target_system}")
            
            self.target_system = self.connection.target_system
            self.target_component = self.connection.target_component
            self.connected = True
            
            return self
        except Exception as e:
            self.logger.info("Connection Failed----")
            self.connected = False
            return None
        
    def set_mode(self):
        pass
    
    def arm(self):
        pass

    def disarm(self):
        pass 

    def take_off(self, altitude=10):
        pass

    def land(self):
        pass

    def get_position(self):
        pass

    

    