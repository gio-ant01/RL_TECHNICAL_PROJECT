#!/usr/bin/env python3
"""
Door Controller Node

Questo nodo controlla l'apertura della porta.
Aspetta il segnale su /pin_inserted dal keypad_controller,
poi apre la porta (e rimane aperta).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String


class DoorController(Node):
    def __init__(self):
        super().__init__('door_controller')

        # Publisher per controllare la porta
        self.door_pub = self.create_publisher(Float64, '/door_slide/cmd_pos', 10)

        # Subscriber per ricevere la notifica di PIN inserito
        self.pin_inserted_sub = self.create_subscription(
            String,
            '/pin_inserted',
            self.pin_inserted_callback,
            10
        )

        # Flag per evitare aperture multiple
        self.door_opened = False

        self.get_logger().info('Door Controller inizializzato')
        self.get_logger().info('In attesa di PIN inserito su /pin_inserted...')

    def pin_inserted_callback(self, msg):
        """Callback chiamata quando il PIN e' stato inserito"""
        if self.door_opened:
            self.get_logger().warn('Porta gia aperta, ignorando richiesta')
            return

        pin = msg.data
        self.get_logger().info(f'PIN "{pin}" inserito correttamente!')
        self.open_door()

    def open_door(self):
        """Apre la porta"""
        self.door_opened = True

        msg = Float64()
        msg.data = 0.9
        self.door_pub.publish(msg)
        self.get_logger().info('=== PORTA APERTA ===')


def main(args=None):
    rclpy.init(args=args)

    door_controller = DoorController()

    try:
        rclpy.spin(door_controller)
    except KeyboardInterrupt:
        door_controller.get_logger().info('Interruzione da tastiera')
    finally:
        door_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()