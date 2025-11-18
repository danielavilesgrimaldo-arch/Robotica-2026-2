
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts 
import sys 

class Cliente(Node):
    def __init__(self):
        super().__init__('cliente_sumador')
        
        
        self.client = self.create_client(AddTwoInts, 'sumar_dos_enteros')
        
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servidor no disponible, esperando...')
        
        
        self.get_logger().info('Servidor encontrado.')

    def enviar_peticion(self, a, b):
        
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        
        self.future = self.client.call_async(request)
        self.get_logger().info(f'Enviando petición: {a} + {b}')

def main(args=None):
    rclpy.init(args=args)
    
    cliente = Cliente()
    
    
    if len(sys.argv) != 3:
        cliente.get_logger().error('Uso: python3 cliente.py <numero1> <numero2>')
        return 
        
    num_a = int(sys.argv[1])
    num_b = int(sys.argv[2])
    
    
    cliente.enviar_peticion(num_a, num_b)
    
    
    while rclpy.ok():
        rclpy.spin_once(cliente) 
        if cliente.future.done():
            try:
                
                response = cliente.future.result()
                cliente.get_logger().info(f'Respuesta recibida: {num_a} + {num_b} = {response.sum}')
            except Exception as e:
                cliente.get_logger().error(f'La llamada al servicio falló: {e}')
            break
            
    cliente.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()