

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts 

class Servidor(Node):
    def __init__(self):
        super().__init__('servidor_sumador')
        
        
        self.srv = self.create_service(
            AddTwoInts, 
            'sumar_dos_enteros', 
            self.callback_del_servicio)
        
        self.get_logger().info('Servidor de sumas iniciado. Listo para recibir peticiones.')

    def callback_del_servicio(self, request, response):
        
        a = request.a
        b = request.b
        
       
        response.sum = a + b
        
        
        self.get_logger().info(f'Petición recibida: {a} + {b} = {response.sum}')
        
        
        return response

def main(args=None):
    rclpy.init(args=args)
    
    servidor = Servidor()
    
   
    rclpy.spin(servidor)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()