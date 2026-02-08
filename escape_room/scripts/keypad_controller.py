#!/usr/bin/env python3
"""
Keypad Controller Node per Armando

Questo nodo riceve un PIN (stringa) sul topic /pin_code e comanda
il braccio robotico armando per digitare la sequenza sul tastierino.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import time


class KeypadController(Node):
    def __init__(self):
        super().__init__('keypad_controller')

        # Publisher per i comandi di posizione
        self.position_pub = self.create_publisher(
            Float64MultiArray,
            '/arm/position_controller/commands',
            10
        )

        # Publisher per notificare quando il PIN è stato inserito
        self.pin_inserted_pub = self.create_publisher(
            String,
            '/pin_inserted',
            10
        )

        # Subscriber per ricevere il PIN
        self.pin_sub = self.create_subscription(
            String,
            '/pin_code',
            self.pin_callback,
            10
        )

        # Tempo di attesa tra i movimenti (secondi)
        self.move_delay = 2.0  # Tempo per raggiungere la posizione
        self.press_delay = 0.5  # Tempo di "pressione" del tasto

        # === POSE CALIBRATE ===
        # Formato: [j0, j1, j2, j3]

        self.HOME = [0.0, -2.0, 1.5, 2.0]

        # Waypoint intermedio per transizioni sicure
        # Posizione "neutra" tra HOME e i tasti
        self.WAYPOINT = [0.0, -0.5, 1.5, 0.5]

        # Pose per ogni tasto del tastierino
        self.KEY_POSES = {
            '0': [0.0, 0.5, 2.0, -0.5],
            '1': [0.32, 0.32, 0.62, 0.44],
            '2': [0.0, 0.28, 0.62, 0.52],
            '3': [-0.3, 0.32, 0.62, 0.44],
            '4': [0.32, 0.12, 1.38, 0.14],
            '5': [0.0, 0.0, 1.57, 0.0],
            '6': [-0.3, 0.12, 1.38, 0.14],
            '7': [0.32, 0.32, 1.88, -0.5],
            '8': [0.0, 0.26, 1.96, -0.58],
            '9': [-0.3, 0.32, 1.88, -0.5],
        }

        # Flag per evitare esecuzioni multiple simultanee
        self.is_busy = False

        self.get_logger().info('Keypad Controller inizializzato')
        self.get_logger().info('In attesa di PIN sul topic /pin_code...')

        # Vai in posizione HOME all'avvio
        self.go_to_home()

    def send_position(self, position):
        """Invia un comando di posizione ai joint"""
        msg = Float64MultiArray()
        msg.data = position
        self.position_pub.publish(msg)
        self.get_logger().debug(f'Posizione inviata: {position}')

    def move_to(self, position, delay=None):
        """Muove il braccio alla posizione specificata e attende"""
        if delay is None:
            delay = self.move_delay
        self.send_position(position)
        time.sleep(delay)

    def go_to_home(self):
        """Porta il braccio in posizione HOME passando dal waypoint"""
        self.get_logger().info('Movimento verso HOME...')
        self.move_to(self.WAYPOINT)
        self.move_to(self.HOME)
        self.get_logger().info('Raggiunta posizione HOME')

    def press_key(self, key):
        """
        Preme un singolo tasto del tastierino.
        Sequenza: HOME -> WAYPOINT -> TASTO -> WAYPOINT -> HOME
        """
        if key not in self.KEY_POSES:
            self.get_logger().warn(f'Tasto non valido: {key}')
            return False

        self.get_logger().info(f'Pressione tasto: {key}')

        # 1. Passa per il waypoint (transizione sicura)
        self.move_to(self.WAYPOINT)

        # 2. Vai alla posizione del tasto
        self.move_to(self.KEY_POSES[key])

        # 3. Breve pausa per "premere" il tasto
        time.sleep(self.press_delay)

        # 4. Torna al waypoint
        self.move_to(self.WAYPOINT)

        # 5. Torna a HOME
        self.move_to(self.HOME)

        self.get_logger().info(f'Tasto {key} premuto con successo')
        return True

    def enter_pin(self, pin):
        """
        Digita l'intero PIN, cifra per cifra.
        """
        self.get_logger().info(f'=== Inizio digitazione PIN: {pin} ===')

        # Assicurati di partire da HOME
        self.go_to_home()

        # Digita ogni cifra
        for i, digit in enumerate(pin):
            self.get_logger().info(f'Cifra {i+1}/{len(pin)}: {digit}')

            if not self.press_key(digit):
                self.get_logger().error(f'Errore nella digitazione della cifra: {digit}')
                return False

        self.get_logger().info(f'=== PIN {pin} digitato con successo! ===')

        # Notifica che il PIN è stato inserito
        pin_msg = String()
        pin_msg.data = pin
        self.pin_inserted_pub.publish(pin_msg)
        self.get_logger().info(f'Notifica PIN inserito inviata su /pin_inserted')

        return True

    def pin_callback(self, msg):
        """Callback chiamata quando si riceve un PIN"""
        pin = msg.data.strip()

        if self.is_busy:
            self.get_logger().warn('Digitazione in corso, PIN ignorato')
            return

        # Valida il PIN (solo cifre)
        if not pin.isdigit():
            self.get_logger().error(f'PIN non valido (deve contenere solo cifre): {pin}')
            return

        self.get_logger().info(f'PIN ricevuto: {pin}')

        self.is_busy = True
        try:
            self.enter_pin(pin)
        finally:
            self.is_busy = False


def main(args=None):
    rclpy.init(args=args)

    node = KeypadController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interruzione da tastiera')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
