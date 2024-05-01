
from ArduinoCommunication import ArduinoCommunication
from SidewalkEdges import sidewalk_edge_detection
import TestCases

if __name__ == "__main__":
    arduinoComm = ArduinoCommunication()
    arduinoComm.start_communication()
    TestCases.vmain(lambda image:sidewalk_edge_detection(image, arduinoComm.write_queue), resize_factor=0.3, video_path=0)
  