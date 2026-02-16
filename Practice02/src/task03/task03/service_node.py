import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        
        self.declare_parameter('service_name', '/trigger_service')
        self.declare_parameter('default_string', 'No service available')
        
        service_name_param = self.get_parameter('service_name').get_parameter_value().string_value
        self.default_string = self.get_parameter('default_string').get_parameter_value().string_value
        self.stored_string = None

        self.client = self.create_client(Trigger, '/spgc/trigger')
        
        self.srv = self.create_service(Trigger, service_name_param, self.trigger_callback)
        self.get_logger().info(f'Providing service at {service_name_param}')

        self.timer = self.create_timer(1.0, self.call_external_service)

    def call_external_service(self):
        if self.stored_string is not None:
            self.timer.cancel()
            return

        if not self.client.service_is_ready():
            self.get_logger().warn('Service /spgc/trigger not available, retrying...')
            return

        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.client_callback)
        self.timer.cancel()

    def client_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.stored_string = response.message
                self.get_logger().info(f'Stored string from /spgc/trigger: {self.stored_string}')
            else:
                self.get_logger().error('External service call returned success=False')
                self.timer.reset()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.timer.reset()

    def trigger_callback(self, request, response):
        if self.stored_string is not None:
            response.success = True
            response.message = self.stored_string
        else:
            response.success = False
            response.message = self.default_string
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
