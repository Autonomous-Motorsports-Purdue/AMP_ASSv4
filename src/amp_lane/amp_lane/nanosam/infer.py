import sys
from typing import Union, Dict
import cv2
from mobile_sam import SamAutomaticMaskGenerator, SamPredictor, sam_model_registry
import numpy as np
import torch
from torchvision.ops import masks_to_boxes
from ament_index_python import get_package_share_directory

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

print("Building SAM model")
mask_generator = SamAutomaticMaskGenerator(
    mobile_sam,
    points_per_side=8,
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
    lowerbox = np.zeros(image.shape[0:2], dtype=bool)
    lowerbox[int(image.shape[0] * 0.85):-1, :] = True

    def mask_score(mask: Union[np.ndarray, Dict]):
        ''' Score each mask on its liklihood of being a road
            Note no color operations here since it's a bit of a pickle
            to deal with white point / exposure

            We use two metrics:
            - How much of the mask intersects with the bottom half of the screen.
              Because the top half usually contains farplane objects which are more
              volatile in terms of size and placement, the lower half is usually a 
              better predictor since it restricts the candidates to nearby objects
            - How centered is the bottom half of the mask. There are some walls and 
              barriers that may take up a large portion of the bottom half of the screen.
              There are scored lower since they're off to the size 
        '''
        if isinstance(mask, dict):
            mask = mask['segmentation']

        intersection = mask * lowerbox
        lower_bbox = masks_to_boxes(torch.from_numpy(intersection).unsqueeze(0))[0]

        # Compute the X center of the bounding box of the bottom of the mask
        lower_bbox_centering = (lower_bbox[0] + lower_bbox[2]) / 2
        # Find its distance from the X center
        lower_bbox_centering = abs(halfwidth - lower_bbox_centering)
        # Flip the distance (to reward centered masks) and normalize the score to [0, 1)
        lower_bbox_centering = (-lower_bbox_centering + halfwidth) / halfwidth
        
        return intersection.sum() / total_area + \
               lower_bbox_centering.item()
    
    print(list(map(mask_score, masks)))
    road_mask = max(masks, key=mask_score)

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
