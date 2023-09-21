import sys
import time
from typing import Union, Dict
import cv2
from mobile_sam import SamAutomaticMaskGenerator, SamPredictor, sam_model_registry
import numpy as np
import torch
from ament_index_python import get_package_share_directory

_last_time = None
def measure_time(name):
    global _last_time
    now = time.time()

    if _last_time == None or name == 'Start Timing':
        print('\n\n')
        _last_time = now
        return
    
    print('\tTiming', name, "=", now - _last_time)
    _last_time = now


def masks_to_boxes(mask):
    """
    Compute the bounding boxes around the provided mask.

    Returns a Tuple of bounding box in ``(x1, y1, x2, y2)`` format with
    ``0 <= x1 < x2`` and ``0 <= y1 < y2``.

    Args:
        masks (NDarray[H, W]): mask to transform (H, W) are the spatial dimensions.

    Returns:
        Tuple[4]: bounding box
    """

    y, x = np.where(mask != 0)

    if x.size == 0 or y.size == 0:
        return (0, 0, 0, 0)

    return (np.min(x),  np.min(y), np.max(x), np.max(y))

def show_anns(anns, orig):
    if len(anns) == 0:
        return
    
    if isinstance(anns[0], dict):
        for i in range(len(anns)):
            anns[i] = anns[i]['segmentation'] 

    sorted_anns = sorted(anns, key=(lambda x: x.sum()), reverse=True)

    # img = np.dstack( [orig * 0.7, np.zeros(orig.shape[0:2])])
    img = (orig * 0.5).astype(np.uint8)

    assert(sorted_anns[0].shape[0] == img.shape[0])
    assert(sorted_anns[0].shape[1] == img.shape[1])
    assert(img.shape[2] == 3)

    g = np.random.Generator(np.random.MT19937(42))
    np.array([0.9, 0.8, 0.2])

    for ann in sorted_anns:
        color_mask = g.random(3) * 255 * 0.5
        img[ann] += (color_mask).astype(np.uint8)
    return img

model_type = "vit_t"
sam_checkpoint = get_package_share_directory('amp_lane') + "/nanosam/mobile_sam.pt"

device = "cuda" if torch.cuda.is_available() else "cpu"

print("Loading SAM model to device", device)
mobile_sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
mobile_sam.to(device=device)
mobile_sam.eval()

def build_lower_point_grid(x_across, y_range) -> np.ndarray:
    """Generates a 2D grid of points evenly spaced in [0,1]x[0,1].
       Use only part of image hwere y>= y_range
    """

    y_across = int(y_range * x_across)

    offset = 1 / (2 * x_across)

    points_x = np.linspace(offset, 1 - offset, x_across)
    points_y = np.linspace(y_range + offset, 1 - offset, y_across)
    
    points_x = np.tile(points_x[None, :], (y_across, 1))
    points_y = np.tile(points_y[:, None], (1, x_across))

    points = np.stack([points_x, points_y], axis=-1).reshape(-1, 2)
    return points

print("Building SAM model")
mask_generator = SamAutomaticMaskGenerator(
    mobile_sam,
    points_per_side=None,
    point_grids=[build_lower_point_grid(6, 0.5)],
    points_per_batch=64,
    )

# predictor = SamPredictor(mobile_sam)

def infer(image: np.ndarray, visualize=True):

    print('Mask gen')
    # predictor.set_image(image)
    # masks, scores, low_res = predictor.predict(
    #     point_coords=np.array([(image.shape[1] * 0.4, image.shape[0] - 1),
    #                            (image.shape[1] * 0.6, image.shape[0] - 1),
    #                            (image.shape[1] // 2, image.shape[0] * 0.9)]),
    #     point_labels=np.array([1, 1, 1]),
    #     multimask_output=False
    # )

    masks = mask_generator.generate(image)
    print('Drawing annotations')

    # Filter Masks
    total_area = image.shape[0] * image.shape[1]
    halfheight = image.shape[0] // 2
    halfwidth = image.shape[1] // 2

    def mask_score(mask: Union[np.ndarray, Dict]):
        ''' Score each mask on its liklihood of being a road
            Note no color operations here since it's a bit of a pickle
            to deal with white point / exposure

            We use these metrics:
            - How much of the mask intersects with the bottom half of the screen.
              Because the top half usually contains farplane objects which are more
              volatile in terms of size and placement, the lower half is usually a 
              better predictor since it restricts the candidates to nearby objects
            - How centered is the bottom half of the mask. There are some walls and 
              barriers that may take up a large portion of the bottom half of the screen.
              There are scored lower since they're off to the size
            - How grayish the color of the mask is 
        '''
        if isinstance(mask, dict):
            mask = mask['segmentation']

        measure_time('Start Timing')

        intersection = mask.copy()
        intersection[0:int(image.shape[0] * 0.5)] = False

        measure_time('intersection_time')

        # Compute the X center of the bounding box of the bottom of the mask
        lower_bbox = masks_to_boxes(intersection)
        lower_bbox_centering = (lower_bbox[0] + lower_bbox[2]) / 2
        # Find its distance from the X center
        lower_bbox_centering = abs(halfwidth - lower_bbox_centering)
        # Flip the distance (to reward centered masks) and normalize the score to [0, 1)
        lower_bbox_centering = (-lower_bbox_centering + halfwidth) / halfwidth

        measure_time('BB time')

        # Compute average HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[..., 2] * mask
        avg_saturation = np.mean(hsv)
        print('avg sat', avg_saturation)
        color_score = (255 - avg_saturation) / 255 # How far the saturation is from zero

        measure_time('Color time')

        intersect_area = intersection.sum() / total_area

        measure_time('Area time')
        
        return intersect_area.item() * \
               lower_bbox_centering.item() * \
               color_score.item()
    
    mask_scores = list(map(mask_score, masks))
    print('Mask Scores:', mask_scores)
    road_mask = masks[mask_scores.index(max(mask_scores))]

    # if mask_score(road_mask) >= 0.5:
    #     masks = [road_mask]
    # else:
    #     masks = []

    if visualize:
        return road_mask, show_anns(masks, image)
    else:
        return road_mask, None

if __name__ == '__main__':
    # Video
    # infer(Image.open('/home/lucy/video/05081544_0305-000001.jpg'))
    # cv2.waitKey(0)

    cap = cv2.VideoCapture(sys.argv[1])

    frameCount = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frameWidth = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frameHeight = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    ret = True

    print(cap.isOpened())

    while ret:
        for _ in range(10):
            cap.read()
        
        ret, buf = cap.read()
        masks, anns = infer(buf)

        cv2.imshow('anns', anns)
        cv2.waitKey(1)

    cap.release()
