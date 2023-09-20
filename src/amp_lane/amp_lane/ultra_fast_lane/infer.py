import numpy as np
import torch, cv2
from amp_lane.ultra_fast_lane.ultra_fast_lane.utils.common import merge_config, get_model
import torchvision.transforms as transforms
from ament_index_python.packages import get_package_share_directory

def pred2coords(pred, row_anchor, col_anchor, local_width = 1, original_image_width = 1640, original_image_height = 590):
    batch_size, num_grid_row, num_cls_row, num_lane_row = pred['loc_row'].shape
    batch_size, num_grid_col, num_cls_col, num_lane_col = pred['loc_col'].shape

    max_indices_row = pred['loc_row'].argmax(1).cpu()
    # n , num_cls, num_lanes
    valid_row = pred['exist_row'].argmax(1).cpu()
    # n, num_cls, num_lanes

    max_indices_col = pred['loc_col'].argmax(1).cpu()
    # n , num_cls, num_lanes
    valid_col = pred['exist_col'].argmax(1).cpu()
    # n, num_cls, num_lanes

    pred['loc_row'] = pred['loc_row'].cpu()
    pred['loc_col'] = pred['loc_col'].cpu()

    coords = []

    row_lane_idx = [1,2]
    col_lane_idx = [0,3]

    for i in row_lane_idx:
        tmp = []
        if valid_row[0,:,i].sum() > num_cls_row / 2:
            for k in range(valid_row.shape[1]):
                if valid_row[0,k,i]:
                    all_ind = torch.tensor(list(range(max(0,max_indices_row[0,k,i] - local_width), min(num_grid_row-1, max_indices_row[0,k,i] + local_width) + 1)))
                    
                    out_tmp = (pred['loc_row'][0,all_ind,k,i].softmax(0) * all_ind.float()).sum() + 0.5
                    out_tmp = out_tmp / (num_grid_row-1) * original_image_width
                    tmp.append((int(out_tmp), int(row_anchor[k] * original_image_height)))
            coords.append(tmp)

    for i in col_lane_idx:
        tmp = []
        if valid_col[0,:,i].sum() > num_cls_col / 4:
            for k in range(valid_col.shape[1]):
                if valid_col[0,k,i]:
                    all_ind = torch.tensor(list(range(max(0,max_indices_col[0,k,i] - local_width), min(num_grid_col-1, max_indices_col[0,k,i] + local_width) + 1)))
                    
                    out_tmp = (pred['loc_col'][0,all_ind,k,i].softmax(0) * all_ind.float()).sum() + 0.5

                    out_tmp = out_tmp / (num_grid_col-1) * original_image_height
                    tmp.append((int(col_anchor[k] * original_image_width), int(out_tmp)))
            coords.append(tmp)

    return coords

torch.backends.cudnn.benchmark = True

print("Get Config")

args, cfg = merge_config(config=get_package_share_directory('amp_lane') + '/configs/culane_res18.py', 
                         test_model=get_package_share_directory('amp_lane') + 'ultra_fast_lane/culane_res18.pth')
cfg.batch_size = 1
print('setting batch_size to 1 for demo generation')

assert cfg.backbone in ['18','34','50','101','152','50next','101next','50wide','101wide']

print("Build Model")

net = get_model(cfg)

print("Load Model")

state_dict = torch.load(cfg.test_model, map_location='cpu')['model']
compatible_state_dict = {}
for k, v in state_dict.items():
    if 'module.' in k:
        compatible_state_dict[k[7:]] = v
    else:
        compatible_state_dict[k] = v

net.load_state_dict(compatible_state_dict, strict=False)
net.eval()

print("Build Tranforms")

img_transforms = transforms.Compose([
    transforms.Resize((int(cfg.train_height / cfg.crop_ratio), cfg.train_width)),
    transforms.ToTensor(),
    transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
])
if cfg.dataset == 'CULane':
    splits = ['imgList.txt'] # ['test0_normal.txt', 'test1_crowd.txt', 'test2_hlight.txt', 'test3_shadow.txt', 'test4_noline.txt', 'test5_arrow.txt', 'test6_curve.txt', 'test7_cross.txt', 'test8_night.txt']
    # datasets = [LaneTestDataset(cfg.data_root,os.path.join(cfg.data_root, split),img_transform = img_transforms, crop_size = cfg.train_height) for split in splits]
    img_w, img_h = 1640, 590
elif cfg.dataset == 'Tusimple':
    splits = ['test.txt']
    # datasets = [LaneTestDataset(cfg.data_root,os.path.join(cfg.data_root, split),img_transform = img_transforms, crop_size = cfg.train_height) for split in splits]
    img_w, img_h = 1280, 720
else:
    raise NotImplementedError

from PIL import Image


def infer(image_np, visualize=True):

    if isinstance(image_np, np.ndarray):
        img = Image.fromarray(image_np)
        vis = image_np.copy()
        img_h, img_w = vis.shape[0], vis.shape[1]
    elif isinstance(image_np, Image.Image):
        img = image_np
        vis =  np.array(img)
        img_w, img_h = img.size[0], img.size[1]
    else:
        raise Exception("Bad image type", type(image_np))

    imgs = img_transforms(img)
    imgs = imgs[None, :,- cfg.train_height:,:]

    imgs = imgs.cuda()
    with torch.no_grad():
        pred = net(imgs)

    coords = pred2coords(pred, cfg.row_anchor, cfg.col_anchor, original_image_width = img_w, original_image_height = img_h)
    
    if visualize:
        for lane in coords:
            for coord in lane:
                cv2.circle(vis, coord, 5, (255, 0, 0), -1)

        cv2.imshow('annotated', vis)
        

    return coords
    
if __name__ == '__main__':
    # Video
    # infer(Image.open('/home/lucy/video/05081544_0305-000001.jpg'))
    # cv2.waitKey(0)

    cap = cv2.VideoCapture('kart.mp4')

    frameCount = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frameWidth = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frameHeight = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    ret = True

    while ret:
        cap.read()
        cap.read()
        cap.read()
        cap.read()
        
        ret, buf = cap.read()
        infer(buf)
        cv2.waitKey(1)

    cap.release()
