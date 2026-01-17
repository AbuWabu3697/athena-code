#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
import threading

from msgs.srv import SetController
from controller_manager_msgs.srv import SwitchController, ListControllers


class ControllerSwitcher(Node):
    def __init__(self):
        super().__init__('controller_switcher')
        
        # Create separate callback groups
        self.service_cb_group = ReentrantCallbackGroup()
        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        
        # Controllers to always keep active or ignore
        self.always_active = ["joint_state_broadcaster"]
        self.ignore_controllers = []
        
        # Lock to prevent concurrent service processing
        self.service_lock = threading.Lock()
        
        # Create service with callback group
        self.srv = self.create_service(
            SetController, 
            'set_controller', 
            self.set_controller_callback,
            callback_group=self.service_cb_group
        )
        
        # Create clients with separate callback group
        self.switch_client = self.create_client(
            SwitchController, 
            '/controller_manager/switch_controller',
            callback_group=self.client_cb_group
        )
        self.list_client = self.create_client(
            ListControllers, 
            '/controller_manager/list_controllers',
            callback_group=self.client_cb_group
        )
        
        # Wait for services with timeout
        self.get_logger().info('Waiting for controller_manager services...')
        services_ready = False
        for _ in range(30):  # Try for 30 seconds
            if (self.switch_client.wait_for_service(timeout_sec=1.0) and 
                self.list_client.wait_for_service(timeout_sec=1.0)):
                services_ready = True
                break
            self.get_logger().warn('Still waiting for controller_manager services...')
        
        if not services_ready:
            self.get_logger().error('Controller manager services not available!')
            raise RuntimeError('Controller manager services timeout')
            
        self.get_logger().info('ControllerSwitcher ready!')

    def set_controller_callback(self, request, response):
        # Use lock to ensure only one service call is processed at a time
        with self.service_lock:
            return self._handle_set_controller(request, response)

    def _handle_set_controller(self, request, response):
        try:
            requested_controllers = list(request.controller_names) if request.controller_names else []
            
            # Empty list means deactivate all (except always_active and ignored)
            if not requested_controllers:
                self.get_logger().info("Empty controller list - will deactivate all except protected controllers")
            else:
                self.get_logger().info(f"Received request to switch to: {requested_controllers}")
            
            # Get list of all controllers
            list_req = ListControllers.Request()
            self.get_logger().debug("Calling list_controllers service...")
            
            # Call and wait for response
            try:
                list_result = self.list_client.call(list_req)
            except Exception as e:
                response.success = False
                response.message = f"Failed to call list_controllers: {str(e)}"
                return response
            
            if list_result is None:
                response.success = False
                response.message = "Failed to get controller list"
                return response
            
            # Extract controller names and their states
            all_controllers = []
            active_controllers = []
            for c in list_result.controller:
                all_controllers.append(c.name)
                if c.state == 'active':
                    active_controllers.append(c.name)
            
            self.get_logger().info(f"Available controllers: {all_controllers}")
            self.get_logger().info(f"Currently active: {active_controllers}")
            
            # Check if all requested controllers exist (only if not empty)
            if requested_controllers:
                missing_controllers = []
                for controller in requested_controllers:
                    if controller not in all_controllers:
                        missing_controllers.append(controller)
                
                if missing_controllers:
                    response.success = False
                    response.message = f"Controllers not found: {missing_controllers}. Available: {all_controllers}"
                    return response
            
            # Build lists for activation/deactivation
            controllers_to_activate = []
            
            # If not empty, add requested controllers that aren't already active
            if requested_controllers:
                for controller in requested_controllers:
                    if controller not in active_controllers:
                        controllers_to_activate.append(controller)
            
            # Always ensure joint_state_broadcaster is active
            if "joint_state_broadcaster" not in active_controllers and "joint_state_broadcaster" not in controllers_to_activate:
                controllers_to_activate.append("joint_state_broadcaster")
            
            # Deactivate logic:
            # - If empty list: deactivate all except always_active and ignored
            # - If controllers specified: deactivate all except requested, always_active, and ignored
            controllers_to_deactivate = []
            for controller in active_controllers:
                if controller in self.always_active or controller in self.ignore_controllers:
                    continue  # Skip protected controllers
                    
                if not requested_controllers or controller not in requested_controllers:
                    controllers_to_deactivate.append(controller)
            
            # Check if already in desired state
            if requested_controllers:
                all_requested_active = all(c in active_controllers for c in requested_controllers)
                if all_requested_active and len(controllers_to_deactivate) == 0:
                    response.success = True
                    response.message = f"Controllers {requested_controllers} already active with correct configuration"
                    self.get_logger().info("Already in desired state")
                    return response
            else:
                # For empty list, check if only protected controllers are active
                if len(controllers_to_deactivate) == 0 and len(controllers_to_activate) == 0:
                    response.success = True
                    response.message = "Only protected controllers are active"
                    self.get_logger().info("Already in desired state")
                    return response
            
            self.get_logger().info(f"Will activate: {controllers_to_activate}")
            self.get_logger().info(f"Will deactivate: {controllers_to_deactivate}")
            
            # Skip if nothing to do
            if not controllers_to_activate and not controllers_to_deactivate:
                response.success = True
                response.message = "No changes needed"
                return response
            
            # Perform the switch
            switch_req = SwitchController.Request()
            switch_req.activate_controllers = controllers_to_activate
            switch_req.deactivate_controllers = controllers_to_deactivate
            switch_req.strictness = SwitchController.Request.BEST_EFFORT
            switch_req.start_asap = True
            switch_req.timeout = Duration(seconds=10).to_msg()
            
            self.get_logger().debug("Calling switch_controller service...")
            
            try:
                switch_result = self.switch_client.call(switch_req)
            except Exception as e:
                response.success = False
                response.message = f"Failed to call switch_controller: {str(e)}"
                return response
            
            if switch_result and switch_result.ok:
                response.success = True
                if requested_controllers:
                    active_list = requested_controllers + self.always_active
                    response.message = f"Successfully activated controllers: {requested_controllers}. Active: {active_list}"
                else:
                    response.message = f"Successfully deactivated all controllers except: {self.always_active + self.ignore_controllers}"
                self.get_logger().info(f"Success! {response.message}")
            else:
                response.success = False
                response.message = "Controller switch failed"
                self.get_logger().error("Switch failed")
                
        except Exception as e:
            self.get_logger().error(f"Error in set_controller_callback: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Exception: {str(e)}"
        
        self.get_logger().info(f"Returning response: success={response.success}, message={response.message}")
        return response


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ControllerSwitcher()
        # Use MultiThreadedExecutor with more threads
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
            
    except Exception as e:
        print(f"Failed to start ControllerSwitcher: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()