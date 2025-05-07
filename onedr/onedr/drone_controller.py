import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping 
# TODO NOT sure why pymavlink has issue iwth MutableMapping; just used older method for now.. check later
from pymavlink import mavutil
from onedr.constants import Mode, ArmState

import time
 

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
    
    def wait_for_position_estimate(self, timeout=60):
        """
        Wait for the drone to get a valid position estimate.
        
        Args:
            timeout (int): Maximum time to wait in seconds
            
        Returns:
            bool: True if position estimate was obtained, False if timed out
        """
        if not self.connected:
            self.logger.info("Not connected to a drone.")
            return False
            
        self.logger.info("Waiting for position estimate...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # Check GPS status
            self.connection.mav.request_data_stream_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                1,  # rate in Hz
                1   # start
            )
            
            msg = self.connection.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
            if msg:
                fix_type = msg.fix_type
                if fix_type >= 3:  # 3D fix or better
                    satellites = msg.satellites_visible
                    self.logger.info(f"Position estimate obtained! GPS fix type: {fix_type}, Satellites: {satellites}")
                    return True
                else:
                    self.logger.info(f"Waiting for better GPS fix. Current fix type: {fix_type}, Satellites: {msg.satellites_visible}")
            
            # Check for EKF status if available
            msg = self.connection.recv_match(type='EKF_STATUS_REPORT', blocking=False)
            if msg:
                flags = msg.flags
                if flags & mavutil.mavlink.EKF_ATTITUDE and flags & mavutil.mavlink.EKF_POS_HORIZ_REL:
                    self.logger.info("EKF reports good position estimate.")
                    return True
                    
            time.sleep(1)
            
        self.logger.info("Timed out waiting for position estimate.")
        return False
         
    def set_mode(self, mode):
        """
        Set the drone's flight mode.
        
        Args:
            mode (str): The flight mode to set (e.g., 'GUIDED', 'LOITER', 'RTL', 'AUTO')
        
        Returns:
            bool: True if the mode was set successfully, False otherwise
        """
        if not self.connected:
            self.logger.info("Not connected to a drone.")
            return False 
        
        if not isinstance(mode, Mode):
            self.logger.error(f"Invalid mode: {mode}. Expected a Mode enum.")
            return False
        
        mode_id = mode.value  # Access the MAVLink mode ID from the enum value
        self.logger.info(f"Setting mode to {mode.name} ({mode.value})...")
        
        self.connection.mav.set_mode_send(
            self.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        
        # Wait for mode change to be acknowledged
        start = time.time()
        while time.time() - start < 3:
            msg = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                current_mode = msg.custom_mode
                if current_mode == mode_id:
                    self.logger.info(f"Mode changed to {mode.name} successfully!")
                    return True
        
        self.logger.error(f"Failed to change mode to {mode.name}")
        return False

    
    def set_guided(self):
        return self.set_mode(Mode.GUIDED)
    
    def arm(self, wait_for_position=True, position_timeout=60):
        """
        Arm the drone's motors.
        
        Args:
            wait_for_position (bool): Whether to wait for position estimate before arming
            position_timeout (int): Maximum time to wait for position in seconds
            
        Returns:
            bool: True if armed successfully, False otherwise
        """
        self.logger.info("Arming motors...")
        
        # Set guided if not already in GUIDED mode; sometimes guided mode automatically turns off
        self.set_mode(Mode.GUIDED)
        time.sleep(1)
        
        # Cannot be armed if position estimate is not available. 
        if wait_for_position:
            position_ready = self.wait_for_position_estimate(timeout=position_timeout) # GPS fix takes some time , wait until GPS is connected.
            if not position_ready:
                self.logger.info("Failed to get position estimate. Arming may fail.")

        return self._send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            ArmState.ARM
        )
    
    def disarm(self):
        """Disarm the drone's motors."""

        self.logger.info("Disarming motors...")
        return self._send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            ArmState.DISARM
        ) 

    def take_off(self, altitude=10):
        """
        Command the drone to take off to a specified altitude.
        
        Args:
            altitude (float): Target altitude in meters
        
        Returns:
            bool: True if the command was accepted, False otherwise
        """
        print(f"Taking off to {altitude} meters...")
        return self._send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, altitude
        )

    def land(self):
        pass

    def get_position(self):
        pass

    def _send_command_long(self, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        """
        Send a COMMAND_LONG message to the drone.
        
        Args:
            command: The MAVLink command to send
            param1-param7: Command parameters
        
        Returns:
            bool: True if the command was acknowledged, False otherwise
        """
        if not self.connected:
            self.logger.info("Not connected to a drone.")
            return False
        
        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            command,
            0,  # confirmation
            param1, param2, param3, param4, param5, param6, param7
        )
        
        # Wait for an acknowledgment
        start = time.time()
        while time.time() - start < 3:
            msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            if msg and msg.command == command:
                if msg.result == 0:
                    self.logger.info(f"Command {command} accepted!")
                    return True
                else:
                    self.logger.info(f"Command {command} failed with result: {msg.result}")
                    return False
        
        self.logger.info(f"No acknowledgment received for command {command}")
        return False

    

    