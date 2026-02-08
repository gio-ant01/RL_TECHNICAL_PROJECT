#!/usr/bin/env python3
"""
ArUco to PIN Bridge Node

Questo nodo fa da ponte tra il rilevamento ArUco di fra2mo e il controllo
del tastierino di armando. Legge l'ID del marker dal topic /marker_publisher/markers
e lo invia come PIN al topic /pin_code dopo averlo rilevato per N frame consecutivi.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from aruco_msgs.msg import MarkerArray


class ArucoToPinBridge(Node):
    def __init__(self):
        super().__init__('aruco_to_pin_bridge')

        # Parametri configurabili
        self.declare_parameter('required_consecutive_frames', 5)
        self.declare_parameter('markers_topic', '/marker_publisher/markers')
        self.declare_parameter('pin_topic', '/pin_code')

        self.required_frames = self.get_parameter('required_consecutive_frames').value
        markers_topic = self.get_parameter('markers_topic').value
        pin_topic = self.get_parameter('pin_topic').value

        # Publisher per inviare il PIN ad armando
        self.pin_pub = self.create_publisher(String, pin_topic, 10)

        # Subscriber per ricevere i marker da fra2mo
        self.marker_sub = self.create_subscription(
            MarkerArray,
            markers_topic,
            self.marker_callback,
            10
        )

        # Stato interno
        self.current_id = None          # ID attualmente in fase di conferma
        self.consecutive_count = 0       # Contatore frame consecutivi
        self.sent_pins = set()           # PIN già inviati (evita duplicati)

        self.get_logger().info('=== ArUco to PIN Bridge avviato ===')
        self.get_logger().info(f'In ascolto su: {markers_topic}')
        self.get_logger().info(f'Pubblicherà su: {pin_topic}')
        self.get_logger().info(f'Frame consecutivi richiesti: {self.required_frames}')

    def marker_callback(self, msg: MarkerArray):
        """Callback chiamata quando si ricevono marker ArUco"""

        # Se non ci sono marker, reset del contatore
        if not msg.markers:
            if self.current_id is not None:
                self.get_logger().debug('Nessun marker rilevato, reset contatore')
            self.current_id = None
            self.consecutive_count = 0
            return

        # Prendi il primo marker rilevato (assumiamo un solo marker sulla porta)
        detected_id = msg.markers[0].id

        # Se l'ID è già stato inviato, ignora
        if detected_id in self.sent_pins:
            return

        # Verifica se è lo stesso ID del frame precedente
        if detected_id == self.current_id:
            self.consecutive_count += 1
            self.get_logger().info(
                f'Marker ID {detected_id} rilevato [{self.consecutive_count}/{self.required_frames}]'
            )
        else:
            # Nuovo ID, reset del contatore
            self.current_id = detected_id
            self.consecutive_count = 1
            self.get_logger().info(
                f'Nuovo marker rilevato: ID {detected_id} [{self.consecutive_count}/{self.required_frames}]'
            )

        # Se abbiamo raggiunto il numero richiesto di frame, invia il PIN
        if self.consecutive_count >= self.required_frames:
            self.send_pin(detected_id)

    def send_pin(self, marker_id: int):
        """Invia il PIN ad armando"""
        pin_str = str(marker_id)

        self.get_logger().info('=' * 50)
        self.get_logger().info(f'MARKER CONFERMATO! ID: {marker_id}')
        self.get_logger().info(f'Invio PIN "{pin_str}" ad armando...')
        self.get_logger().info('=' * 50)

        # Pubblica il PIN
        msg = String()
        msg.data = pin_str
        self.pin_pub.publish(msg)

        # Segna come inviato per evitare duplicati
        self.sent_pins.add(marker_id)

        # Reset dello stato
        self.current_id = None
        self.consecutive_count = 0

    def reset_sent_pins(self):
        """Metodo per resettare i PIN inviati (utile per test)"""
        self.sent_pins.clear()
        self.get_logger().info('Lista PIN inviati resettata')


def main(args=None):
    rclpy.init(args=args)

    node = ArucoToPinBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interruzione da tastiera')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
