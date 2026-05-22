import os
import cv2
import rclpy

from rclpy.node import Node
from std_msgs.msg import String

from .sign_vision_core import (
    detect_papers,
    has_signal_content,
    load_reference_descriptors,
    classify_signal
)


class SignDetectorDirectNode(Node):
    def __init__(self):
        super().__init__("sign_detector_direct_node")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("publish_topic", "/detected_signal")
        self.declare_parameter("min_score", 0.10)
        self.declare_parameter("process_fps", 5.0)
        self.declare_parameter("stable_frames", 4)
        self.declare_parameter("debug", False)

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.publish_topic = self.get_parameter("publish_topic").value
        self.min_score = float(self.get_parameter("min_score").value)
        self.process_fps = float(self.get_parameter("process_fps").value)
        self.stable_frames = int(self.get_parameter("stable_frames").value)
        self.debug = bool(self.get_parameter("debug").value)

        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.reference_folder = os.path.join(current_dir, "reference")

        self.get_logger().info(f"Cargando referencias desde: {self.reference_folder}")
        self.references = load_reference_descriptors(self.reference_folder)

        if len(self.references) == 0:
            self.get_logger().error("No se cargaron referencias. Revisa la carpeta reference/.")
        else:
            self.get_logger().info(f"Clases cargadas: {list(self.references.keys())}")

        self.publisher = self.create_publisher(String, self.publish_topic, 10)

        self.cap = cv2.VideoCapture(self.camera_index)

        if not self.cap.isOpened():
            self.get_logger().error(f"No se pudo abrir la cámara index={self.camera_index}")
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.last_label = "unknown"
        self.same_label_count = 0
        self.last_published_label = "unknown"

        timer_period = 1.0 / self.process_fps
        self.timer = self.create_timer(timer_period, self.process_frame)

        self.get_logger().info("Nodo directo de detección iniciado.")
        self.get_logger().info(f"Cámara: /dev/video{self.camera_index}")
        self.get_logger().info(f"Publicando en: {self.publish_topic}")

    def process_frame(self):
        if not hasattr(self, "cap") or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.get_logger().warn("No se pudo leer frame de la cámara.")
            return

        label, score = self.detect_best_signal(frame)

        if label == self.last_label:
            self.same_label_count += 1
        else:
            self.last_label = label
            self.same_label_count = 1

        if self.same_label_count >= self.stable_frames:
            if label != self.last_published_label and label != "unknown":
                self.publish_detection(label, score)
                self.last_published_label = label

            elif label == "unknown" and self.last_published_label != "unknown":
                self.publish_detection("unknown", score)
                self.last_published_label = "unknown"

    def detect_best_signal(self, frame):
        papers, white_mask, display = detect_papers(frame)

        best_label = "unknown"
        best_score = 0.0

        for paper in papers:
            if len(paper) == 5:
                x, y, w, h, area = paper
            else:
                x, y, w, h = paper

            crop = display[y:y + h, x:x + w]

            is_signal, ratios = has_signal_content(crop)

            if not is_signal:
                continue

            label, score, class_scores = classify_signal(
                crop,
                self.references,
                debug=self.debug
            )

            if score < self.min_score:
                label = "unknown"

            if score > best_score:
                best_score = score
                best_label = label

        if self.debug:
            self.get_logger().info(f"Detection: {best_label} score={best_score:.3f}")

        return best_label, best_score

    def publish_detection(self, label, score):
        msg = String()
        msg.data = f"{label},{score:.3f}"
        self.publisher.publish(msg)
        self.get_logger().info(f"Publicado: {msg.data}")

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SignDetectorDirectNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
