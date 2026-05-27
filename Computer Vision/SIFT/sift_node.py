import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import os
import numpy as np

from ament_index_python.packages import get_package_share_directory


class SIFTSignalNode(Node):

    def __init__(self):
        super().__init__('sift_signal_node')

        self.publisher = self.create_publisher(String, '/detected_signal', 10)

        package_path = get_package_share_directory('integration_test_1')
        self.folder = os.path.join(package_path, 'fotos')

        self.database = self.load_database()

        self.sift = cv2.SIFT_create()
        self.bf = cv2.BFMatcher()

        self.cap = cv2.VideoCapture(0)

        self.last_label = "unknown"
        self.same_count = 0

        self.timer = self.create_timer(0.1, self.process_frame)

    def load_database(self):
        database = {}

        for class_name in os.listdir(self.folder):
            class_path = os.path.join(self.folder, class_name)

            if not os.path.isdir(class_path):
                continue

            database[class_name] = []

            for file in os.listdir(class_path):
                path = os.path.join(class_path, file)

                img = cv2.imread(path)

                if img is None:
                    continue

                img = cv2.resize(img, (240, 240))
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                kp, des = cv2.SIFT_create().detectAndCompute(gray, None)

                if des is not None:
                    database[class_name].append(des)

        return database

    def has_signal_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask_red1 = cv2.inRange(hsv, (0, 80, 60), (10, 255, 255))
        mask_red2 = cv2.inRange(hsv, (170, 80, 60), (180, 255, 255))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        mask_blue = cv2.inRange(hsv, (90, 80, 50), (130, 255, 255))
        mask_yellow = cv2.inRange(hsv, (15, 80, 80), (40, 255, 255))
        mask_green = cv2.inRange(hsv, (40, 60, 50), (85, 255, 255))

        total = frame.shape[0] * frame.shape[1]

        color_ratio = (
            cv2.countNonZero(mask_red) +
            cv2.countNonZero(mask_blue) +
            cv2.countNonZero(mask_yellow) +
            cv2.countNonZero(mask_green)
        ) / total

        return color_ratio > 0.02

    def classify(self, frame):
        if not self.has_signal_color(frame):
            return "unknown", 0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, (240, 240))

        kp_test, des_test = self.sift.detectAndCompute(gray, None)

        if des_test is None:
            return "unknown", 0

        best_label = "unknown"
        best_score = 0

        for label in self.database:
            total_matches = 0

            for des_ref in self.database[label]:
                matches = self.bf.knnMatch(des_ref, des_test, k=2)

                good = []
                for m, n in matches:
                    if m.distance < 0.75 * n.distance:
                        good.append(m)

                total_matches += len(good)

            if total_matches > best_score:
                best_score = total_matches
                best_label = label

        MIN_MATCHES = 15

        if best_label == "forbidden":
            if best_score < 8:
                best_label = "unknown"
        else:
            if best_score < MIN_MATCHES:
                best_label = "unknown"

        if best_label in ["agv_area", "loading"] and best_score < 20:
            best_label = "unknown"

        return best_label, best_score

    def process_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            return

        label, score = self.classify(frame)

        if label == self.last_label:
            self.same_count += 1
        else:
            self.same_count = 1
            self.last_label = label

        if self.same_count < 3:
            label = "unknown"

        msg = String()
        msg.data = f"{label},{score}"
        self.publisher.publish(msg)

        display = frame.copy()

        text = f"{label} ({score})"
        cv2.putText(display, text, (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 255, 0) if label != "unknown" else (0, 0, 255), 2)

        cv2.imshow("SIFT Detection", display)
        cv2.waitKey(1)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SIFTSignalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
