import sys
import rclpy
from rclpy.node import Node

# Import RFM9x
import adafruit_rfm9x

class LoRaNode(Node):

    def __init__(self):
        super().__init__('lora_node')

        # Configure LoRa Radio
        CS = DigitalInOut(board.CE1)
        RESET = DigitalInOut(board.D25)
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        self.rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915.0)
        self.rfm9x.tx_power = 23

        # Create publisher for northing and easting coordinates
        self.coord_publisher = self.create_publisher(NorthingEasting, 'northing_easting', 10)

        # Start listening for LoRa messages
        self.receive_loop()

    def receive_loop(self):
        while True:
            # Wait for a packet to be received
            packet = self.rfm9x.receive(timeout=10)
            if packet is not None:
                try:
                    received_message = packet.decode("utf-8")
                    northing, easting = map(float, received_message.split(','))
                    self.publish_coordinates(northing, easting)
                except Exception as e:
                    # Publish error message
                    self.get_logger().error(f"Error receiving message: {e}")
                    self.publish_error("Error receiving coordinates. Rover stopped.")
                print("Received coordinates:", northing, easting)

    def publish_coordinates(self, northing, easting):
        msg = NorthingEasting()
        msg.northing = northing
        msg.easting = easting
        self.coord_publisher.publish(msg)

    def publish_error(self, message):
        # Publish error message
        msg = String()
        msg.data = message
        self.error_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    lora_node = LoRaNode()
    rclpy.spin(lora_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
