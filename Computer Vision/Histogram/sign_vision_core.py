import os
import cv2
import numpy as np

# ------------------------------------------------------------
# 1. DETECCIÓN DE HOJAS BLANCAS VERTICALES
# ------------------------------------------------------------

def detect_papers(frame):

    frame_resized = cv2.resize(frame, (640, 480))
    blurred = cv2.GaussianBlur(frame_resized, (5, 5), 0)

    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 120])
    upper_white = np.array([180, 95, 255])

    white_mask = cv2.inRange(hsv, lower_white, upper_white)

    kernel = np.ones((7, 7), np.uint8)

    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel, iterations=3)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(
        white_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    papers = []
    frame_area = frame_resized.shape[0] * frame_resized.shape[1]

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < 2500:
            continue

        perimeter = cv2.arcLength(contour, True)

        if perimeter == 0:
            continue

        approx = cv2.approxPolyDP(contour, 0.025 * perimeter, True)

        if len(approx) < 4 or len(approx) > 8:
            continue

        x, y, w, h = cv2.boundingRect(contour)

        if w == 0 or h == 0:
            continue

        bbox_area = w * h

        if bbox_area > 0.80 * frame_area:
            continue

        # Solo verticales
        if h <= w:
            continue

        aspect_ratio = h / float(w)

        # A4 vertical \u2248 1.414, pero permisivo por perspectiva
        if not (1.10 <= aspect_ratio <= 2.20):
            continue

        rectangularity = area / float(w * h)

        if rectangularity < 0.45:
            continue

        papers.append((x, y, w, h, area))

    papers.sort(key=lambda p: p[4], reverse=True)

    return papers, white_mask, frame_resized


# ------------------------------------------------------------
# 2. PREPROCESAMIENTO
# ------------------------------------------------------------

def crop_inner_region(image):
    """
    Recorta la zona interna de la hoja para reducir márgenes blancos.
    """

    if image is None or image.size == 0:
        return None

    h, w = image.shape[:2]

    if h < 20 or w < 20:
        return None

    y1 = int(0.10 * h)
    y2 = int(0.90 * h)
    x1 = int(0.10 * w)
    x2 = int(0.90 * w)

    roi = image[y1:y2, x1:x2]

    if roi.size == 0:
        return None

    return roi


def simple_white_balance(image):
    """
    Balance de blancos simple tipo gray-world.
    Reduce tintes de la cámara.
    """

    result = image.astype(np.float32)

    avg_b = np.mean(result[:, :, 0])
    avg_g = np.mean(result[:, :, 1])
    avg_r = np.mean(result[:, :, 2])

    avg_gray = (avg_b + avg_g + avg_r) / 3.0

    if avg_b > 0:
        result[:, :, 0] *= avg_gray / avg_b
    if avg_g > 0:
        result[:, :, 1] *= avg_gray / avg_g
    if avg_r > 0:
        result[:, :, 2] *= avg_gray / avg_r

    result = np.clip(result, 0, 255).astype(np.uint8)

    return result


# ------------------------------------------------------------
# 3. FILTRO DE CONTENIDO DE SEÑAL
# ------------------------------------------------------------

def has_signal_content(crop, debug=False):
    """
    Revisa si el rectángulo blanco contiene colores fuertes de señal.
    No clasifica todavía.
    """

    roi = crop_inner_region(crop)

    if roi is None:
        return False, {}

    roi = simple_white_balance(roi)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    red1 = cv2.inRange(hsv, np.array([0, 80, 60]), np.array([12, 255, 255]))
    red2 = cv2.inRange(hsv, np.array([168, 80, 60]), np.array([180, 255, 255]))
    mask_red = cv2.bitwise_or(red1, red2)

    mask_yellow = cv2.inRange(hsv, np.array([18, 80, 80]), np.array([42, 255, 255]))
    mask_blue = cv2.inRange(hsv, np.array([85, 60, 50]), np.array([135, 255, 255]))
    mask_green = cv2.inRange(hsv, np.array([40, 60, 50]), np.array([90, 255, 255]))
    mask_black = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 55]))

    total_pixels = roi.shape[0] * roi.shape[1]

    red_ratio = cv2.countNonZero(mask_red) / total_pixels
    yellow_ratio = cv2.countNonZero(mask_yellow) / total_pixels
    blue_ratio = cv2.countNonZero(mask_blue) / total_pixels
    green_ratio = cv2.countNonZero(mask_green) / total_pixels
    black_ratio = cv2.countNonZero(mask_black) / total_pixels

    color_ratio = red_ratio + yellow_ratio + blue_ratio + green_ratio
    dominant_color = max(red_ratio, yellow_ratio, blue_ratio, green_ratio)

    ratios = {
        "red": red_ratio,
        "yellow": yellow_ratio,
        "blue": blue_ratio,
        "green": green_ratio,
        "black": black_ratio,
        "color": color_ratio,
        "dominant_color": dominant_color
    }

    has_clear_color = color_ratio > 0.030
    has_dominant_color = dominant_color > 0.020
    has_supporting_black = black_ratio > 0.020

    is_signal = has_clear_color or (has_dominant_color and has_supporting_black)

    if debug:
        print("Content ratios:", ratios)

    return is_signal, ratios


# ------------------------------------------------------------
# 4. DESCRIPTORES: COLOR SIN BLANCO + FORMA SIN BLANCO
# ------------------------------------------------------------

def make_non_white_mask(hsv):
    """
    Máscara para excluir blanco/gris claro.
    Conserva colores saturados y zonas oscuras.
    """

    saturated = cv2.inRange(
        hsv,
        np.array([0, 35, 0]),
        np.array([180, 255, 255])
    )

    dark = cv2.inRange(
        hsv,
        np.array([0, 0, 0]),
        np.array([180, 255, 90])
    )

    mask = cv2.bitwise_or(saturated, dark)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    return mask


def make_signal_shape_mask(image):
    """
    Crea máscara binaria del contenido no blanco.
    Sirve para comparar forma/patrón interno.
    """

    roi = crop_inner_region(image)

    if roi is None:
        return None

    roi = cv2.resize(roi, (200, 280))
    roi = simple_white_balance(roi)

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    mask = make_non_white_mask(hsv)

    return mask


def compute_histogram_without_white(image):
    """
    Histograma HSV ignorando blanco.
    """

    roi = crop_inner_region(image)

    if roi is None:
        return None

    roi = cv2.resize(roi, (200, 280))
    roi = simple_white_balance(roi)

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    non_white_mask = make_non_white_mask(hsv)

    non_white_pixels = cv2.countNonZero(non_white_mask)

    if non_white_pixels < 50:
        return None

    hist = cv2.calcHist(
        [hsv],
        [0, 1],
        non_white_mask,
        [36, 32],
        [0, 180, 0, 256]
    )

    cv2.normalize(hist, hist, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)

    return hist


def compute_descriptor(image):
    """
    Descriptor combinado:
    - Histograma HSV sin blanco
    - Máscara de forma sin blanco
    """

    hist = compute_histogram_without_white(image)
    shape_mask = make_signal_shape_mask(image)

    if hist is None or shape_mask is None:
        return None

    return {
        "hist": hist,
        "shape_mask": shape_mask
    }


# ------------------------------------------------------------
# 5. CARGAR REFERENCIAS DESDE SUBCARPETAS
# ------------------------------------------------------------

def load_reference_descriptors(reference_folder):
    """
    Carga referencias desde:
        reference/clase/imagen.jpg

    Regresa:
        references = {
            "stop": [descriptor1, descriptor2, ...],
            "forbidden": [descriptor1, descriptor2, ...],
            ...
        }
    """

    references = {}

    if not os.path.exists(reference_folder):
        print(f"No existe el folder: {reference_folder}")
        return references

    valid_extensions = (".jpg", ".jpeg", ".png", ".bmp")

    for class_name in os.listdir(reference_folder):
        class_path = os.path.join(reference_folder, class_name)

        if not os.path.isdir(class_path):
            continue

        descriptors = []

        for filename in os.listdir(class_path):
            if not filename.lower().endswith(valid_extensions):
                continue

            image_path = os.path.join(class_path, filename)
            image = cv2.imread(image_path)

            if image is None:
                print(f"No se pudo leer: {image_path}")
                continue

            descriptor = compute_descriptor(image)

            if descriptor is None:
                print(f"No se pudo calcular descriptor para: {image_path}")
                continue

            descriptors.append(descriptor)

        if len(descriptors) > 0:
            references[class_name] = descriptors
            print(f"Clase cargada: {class_name} ({len(descriptors)} referencias)")
        else:
            print(f"Clase sin referencias válidas: {class_name}")

    return references


# ------------------------------------------------------------
# 6. CLASIFICACIÓN
# ------------------------------------------------------------

def mask_similarity(mask_a, mask_b):
    """
    Compara dos máscaras binarias usando IoU.
    """

    if mask_a is None or mask_b is None:
        return 0.0

    if mask_a.shape != mask_b.shape:
        mask_b = cv2.resize(mask_b, (mask_a.shape[1], mask_a.shape[0]))

    a = (mask_a > 0).astype(np.uint8)
    b = (mask_b > 0).astype(np.uint8)

    intersection = np.logical_and(a, b).sum()
    union = np.logical_or(a, b).sum()

    if union == 0:
        return 0.0

    return intersection / union


def compare_descriptors(candidate_desc, reference_desc):
    """
    Compara candidato contra una referencia.
    Combina color y forma.
    """

    hist_score = cv2.compareHist(
        candidate_desc["hist"],
        reference_desc["hist"],
        cv2.HISTCMP_CORREL
    )

    hist_score_norm = (hist_score + 1.0) / 2.0

    shape_score = mask_similarity(
        candidate_desc["shape_mask"],
        reference_desc["shape_mask"]
    )

    final_score = 0.50 * hist_score_norm + 0.50 * shape_score

    return final_score, hist_score_norm, shape_score


def classify_signal(crop, references, debug=False):
    """
    Clasifica comparando contra múltiples referencias por clase.
    Para cada clase se queda con el mejor score individual.
    """

    candidate_desc = compute_descriptor(crop)

    if candidate_desc is None:
        return "unknown", 0.0, {}

    best_label = "unknown"
    best_score = -999.0

    class_scores = {}

    for class_name, descriptors in references.items():
        best_class_score = -999.0
        best_class_hist = 0.0
        best_class_shape = 0.0

        for ref_desc in descriptors:
            final_score, hist_score, shape_score = compare_descriptors(
                candidate_desc,
                ref_desc
            )

            if final_score > best_class_score:
                best_class_score = final_score
                best_class_hist = hist_score
                best_class_shape = shape_score

        class_scores[class_name] = {
            "final": best_class_score,
            "hist": best_class_hist,
            "shape": best_class_shape
        }

        if best_class_score > best_score:
            best_score = best_class_score
            best_label = class_name

    if debug:
        print("\nScores:")
        sorted_scores = sorted(
            class_scores.items(),
            key=lambda item: item[1]["final"],
            reverse=True
        )

        for class_name, scores in sorted_scores:
            print(
                f"  {class_name}: "
                f"final={scores['final']:.3f}, "
                f"hist={scores['hist']:.3f}, "
                f"shape={scores['shape']:.3f}"
            )

    return best_label, best_score, class_scores



