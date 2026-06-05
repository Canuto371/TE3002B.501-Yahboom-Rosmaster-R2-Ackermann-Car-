import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import os
import numpy as np

from ament_index_python.packages import get_package_share_directory

L_OFFSET = 26
S_MAX = 24
MIN_AREA = 3400
MIN_RECTANGULARITY = 0.63
CLOSE_KERNEL_SIZE = 9
MIN_ORIGINAL_L = 185

class SIFTSignalNode(Node):

    def __init__(self):
        super().__init__('sift_signal_node')

        self.publisher = self.create_publisher(
            String,
            '/detected_signal',
            10
        )

        package_path = get_package_share_directory(
            'integration_test_4'
        )

        self.folder = os.path.join(package_path, 'fotos')

        self.database = self.load_database()

        self.sift = cv2.SIFT_create()
        self.bf = cv2.BFMatcher()

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

        self.last_label = "unknown"
        self.same_count = 0

        self.timer = self.create_timer(
            0.3,
            self.process_frame
        )

        # ROI visualization
        self.last_roi_boxes = []

    # ==========================================================
    # LOAD DATABASE
    # ==========================================================

    def load_database(self):

        database = {}

        for class_name in os.listdir(self.folder):

            class_path = os.path.join(
                self.folder,
                class_name
            )

            if not os.path.isdir(class_path):
                continue

            database[class_name] = []

            for file in os.listdir(class_path):

                path = os.path.join(class_path, file)

                img = cv2.imread(path)

                if img is None:
                    continue

                img = cv2.resize(img, (240, 240))

                gray = cv2.cvtColor(
                    img,
                    cv2.COLOR_BGR2GRAY
                )

                kp, des = cv2.SIFT_create().detectAndCompute(
                    gray,
                    None
                )

                if des is not None:
                    database[class_name].append(des)

        return database

    # ==========================================================
    # COLOR MASK
    # ==========================================================

    def get_color_mask(self, frame):

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # RED
        mask_red1 = cv2.inRange(
            hsv,
            (0, 80, 60),
            (10, 255, 255)
        )

        mask_red2 = cv2.inRange(
            hsv,
            (170, 80, 60),
            (180, 255, 255)
        )

        mask_red = cv2.bitwise_or(
            mask_red1,
            mask_red2
        )

        # BLUE
        mask_blue = cv2.inRange(
            hsv,
            (90, 80, 50),
            (130, 255, 255)
        )

        # YELLOW
        mask_yellow = cv2.inRange(
            hsv,
            (15, 80, 80),
            (40, 255, 255)
        )

        # GREEN
        mask_green = cv2.inRange(
            hsv,
            (40, 60, 50),
            (85, 255, 255)
        )

        combined_mask = (
            mask_red |
            mask_blue |
            mask_yellow |
            mask_green
        )

        # Morphological cleanup
        kernel = np.ones((5, 5), np.uint8)

        combined_mask = cv2.morphologyEx(
            combined_mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        combined_mask = cv2.morphologyEx(
            combined_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        return combined_mask

    # ==========================================================
    # ROI EXTRACTION
    # ==========================================================

    def extract_main_rectangle_roi(self, frame):

        blurred = cv2.GaussianBlur(frame, (5, 5), 0)

        hsv = cv2.cvtColor(
            blurred,
            cv2.COLOR_BGR2HSV
        )

        lab = cv2.cvtColor(
            blurred,
            cv2.COLOR_BGR2LAB
        )

        _, s_ch, _ = cv2.split(hsv)
        l_ch, _, _ = cv2.split(lab)

        clahe = cv2.createCLAHE(
            clipLimit=2.0,
            tileGridSize=(8, 8)
        )

        l_enhanced = clahe.apply(l_ch)

        otsu_value, _ = cv2.threshold(
            l_enhanced,
            0,
            255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )

        threshold_value = int(
            np.clip(
                otsu_value + L_OFFSET,
                0,
                255
            )
        )

        _, mask_bright = cv2.threshold(
            l_enhanced,
            threshold_value,
            255,
            cv2.THRESH_BINARY
        )

        mask_low_sat = cv2.inRange(
            s_ch,
            0,
            S_MAX
        )

        mask_real_bright = cv2.inRange(
            l_ch,
            MIN_ORIGINAL_L,
            255
        )

        mask = cv2.bitwise_and(
            mask_bright,
            mask_low_sat
        )

        mask = cv2.bitwise_and(
            mask,
            mask_real_bright
        )

        close_kernel = np.ones(
            (CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE),
            np.uint8
        )

        open_kernel = np.ones(
            (3, 3),
            np.uint8
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            close_kernel,
            iterations=2
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            open_kernel,
            iterations=1
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        candidates = []

        frame_area = frame.shape[0] * frame.shape[1]

        for contour in contours:

            area = cv2.contourArea(contour)

            if area < MIN_AREA:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            if w == 0 or h == 0:
                continue

            bbox_area = w * h

            if bbox_area > 0.85 * frame_area:
                continue

            if h <= w:
                continue

            rectangularity = area / float(bbox_area)

            if rectangularity < MIN_RECTANGULARITY:
                continue

            perimeter = cv2.arcLength(
                contour,
                True
            )

            if perimeter == 0:
                continue

            approx = cv2.approxPolyDP(
                contour,
                0.025 * perimeter,
                True
            )

            if len(approx) < 4 or len(approx) > 12:
                continue

            candidates.append(
                (area, x, y, w, h)
            )

        self.last_roi_boxes = []

        if len(candidates) == 0:
            return None

        candidates.sort(
            key=lambda item: item[0],
            reverse=True
        )

        _, x, y, w, h = candidates[0]

        padding = 25

        x1 = max(x - padding, 0)
        y1 = max(y - padding, 0)

        x2 = min(
            x + w + padding,
            frame.shape[1]
        )

        y2 = min(
            y + h + padding,
            frame.shape[0]
        )

        self.last_roi_boxes.append(
            (x1, y1, x2, y2)
        )

        roi = frame[y1:y2, x1:x2]

        return roi

    # ==========================================================
    # CLASSIFICATION
    # ==========================================================

    def classify(self, frame):

        # Obtener ROI del rectángulo blanco más grande
        roi = self.extract_main_rectangle_roi(frame)

        if roi is None:
            return "unknown", 0
        #else:
        #    cv2.imshow("ROI", roi)

    # Convertir a escala de grises
        gray = cv2.cvtColor(
            roi,
            cv2.COLOR_BGR2GRAY
        )

        gray = cv2.resize(
            gray,
            (240, 240)
        )

        kp_test, des_test = self.sift.detectAndCompute(
            gray,
            None
        )

        if des_test is None:
            return "unknown", 0

        best_label = "unknown"
        best_score = 0

    # Comparar contra toda la base de datos
        for label in self.database:

            total_matches = 0

            for des_ref in self.database[label]:

                matches = self.bf.knnMatch(
                    des_ref,
                    des_test,
                    k=2
                )

                good = []

                for pair in matches:

                    if len(pair) < 2:
                        continue

                    m, n = pair

                    if m.distance < 0.75 * n.distance:
                        good.append(m)

                total_matches += len(good)

            if total_matches > best_score:

                best_score = total_matches
                best_label = label

    # ======================================================
    # THRESHOLDS
    # ======================================================

        MIN_MATCHES = 15

        if best_label == "forbidden":

            if best_score < 8:
                best_label = "unknown"

        else:

            if best_score < MIN_MATCHES:
                best_label = "unknown"

        if best_label in ["agv_area", "loading"]:

            if best_score < 20:
                best_label = "unknown"

        return best_label, best_score

    # ==========================================================
    # PROCESS FRAME
    # ==========================================================

    def process_frame(self):

        ret, frame = self.cap.read()

        if not ret:
            return

        label, score = self.classify(frame)

        # Temporal filtering
        if label == self.last_label:

            self.same_count += 1

        else:

            self.same_count = 1
            self.last_label = label

        if self.same_count < 3:
            label = "unknown"

        # Publish
        msg = String()
        msg.data = f"{label},{score}"

        self.publisher.publish(msg)

        # ======================================================
        # DISPLAY
        # ======================================================

        display = frame.copy()

        # Draw ROI box
        for box in self.last_roi_boxes:

            x1, y1, x2, y2 = box

            cv2.rectangle(
                display,
                (x1, y1),
                (x2, y2),
                (255, 0, 0),
                2
            )

        text = f"{label} ({score})"

        cv2.putText(
            display,
            text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0) if label != "unknown"
            else (0, 0, 255),
            2
        )

        cv2.imshow("SIFT Detection", display)

        cv2.waitKey(1)

    # ==========================================================
    # CLEANUP
    # ==========================================================

    def destroy_node(self):

        if self.cap.isOpened():
            self.cap.release()

        cv2.destroyAllWindows()

        super().destroy_node()


# ==============================================================
# MAIN
# ==============================================================

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
