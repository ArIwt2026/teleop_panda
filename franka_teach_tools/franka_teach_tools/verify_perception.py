import rclpy
from rclpy.node import Node
import numpy as np
import time
import cv2
from geometry_msgs.msg import PoseStamped
from .listener import LiveStateListener, Spinner

class PerceptionVerifier(Node):
    def __init__(self):
        super().__init__('perception_verifier')
        self.listener = LiveStateListener()
        self.spinner = Spinner(self.listener)
        self.spinner.start()
        self.history = []
        
    def run(self):
        print("\n=== PERCEPTION VERIFIER (STABILITY TEST) ===")
        print("Move the arm! We want DRIFT to stay as small as possible.")
        print("------------------------------------------------------------")
        
        try:
            while rclpy.ok():
                try:
                    # Scan for marker
                    marker_pose = self.listener.detect_marker_pose_base(timeout_sec=0.1, show_window=True, auto_confirm=True)
                    
                    pos = [marker_pose.pose.position.x, marker_pose.pose.position.y, marker_pose.pose.position.z]
                    self.history.append(pos)
                    if len(self.history) > 100:
                        self.history.pop(0)
                        
                    h = np.array(self.history)
                    std = np.std(h, axis=0) * 1000.0 # in mm
                    drift = np.linalg.norm(np.max(h, axis=0) - np.min(h, axis=0)) * 1000.0
                    
                    print(f"\rMARKER: [{pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}] | DRIFT: {drift:5.1f}mm | STD: {np.mean(std):4.1f}mm", end="")
                except Exception as e:
                    print(f"\rSTATUS: [ SEARCHING... ]                                                                    ", end="")
                
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            try: cv2.destroyAllWindows()
            except: pass
            self.spinner.stop()

def main():
    rclpy.init()
    node = PerceptionVerifier()
    node.run()

if __name__ == '__main__':
    main()
