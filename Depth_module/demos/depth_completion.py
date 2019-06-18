import glob
import os
import sys
import time

import cv2
import numpy as np
import png

sys.path.append('../')
from ip_basic import depth_map_utils
from ip_basic import vis_utils
from PIL import Image
from multiprocessing import Process

def show_image(rgb_file):
    Mat = cv2.imread(rgb_file)
    cv2.imshow("Depth completed IPbasic",Mat)
    if cv2.waitKey(1)& 0xFF ==ord('q'):
        return 

def main():
    """Depth maps are saved to the 'outputs' folder.
    """

    ##############################
    # Options
    ##############################
    # Validation set
    #input_depth_dir = os.path.expanduser(
     #   '~/Kitti/depth/val_selection_cropped/velodyne_raw')
    input_depth_dir = os.path.expanduser(
        '../../data/live_feed/velodyne_raw')
    data_split = 'val'

    # Test set
    # input_depth_dir = os.path.expanduser(
    #     '~/Kitti/depth/test_depth_completion_anonymous/velodyne_raw')
    # data_split = 'test'

    # Fast fill with Gaussian blur @90Hz (paper result)
    fill_type = 'fast'
    extrapolate = True
    blur_type = 'gaussian'

    # Fast Fill with bilateral blur, no extrapolation @87Hz (recommended)
    #fill_type = 'fast'
    #extrapolate = False
    #blur_type = 'bilateral'

    # Multi-scale dilations with extra noise removal, no extrapolation @ 30Hz
    #fill_type = 'multiscale'
    #extrapolate = True
    #blur_type = 'bilateral'

    # Save output to disk or show process
    save_output = True

    ##############################
    # Processing
    ##############################
    if save_output:
        # Save to Disk
        show_process = False
        save_depth_maps = True
    else:
        if fill_type == 'fast':
            raise ValueError('"fast" fill does not support show_process')

        # Show Process
        show_process = True
        save_depth_maps = False

    # Create output folder
    this_file_path = os.path.dirname(os.path.realpath(__file__))
    outputs_dir = this_file_path + '/outputs'
    results_dir = this_file_path + '/result'
    os.makedirs(outputs_dir, exist_ok=True)
    os.makedirs(results_dir, exist_ok=True)

    output_folder_prefix = 'depth_' + data_split
    output_list = sorted(os.listdir(outputs_dir))
    if len(output_list) > 0:
        split_folders = [folder for folder in output_list
                         if folder.startswith(output_folder_prefix)]
        if len(split_folders) > 0:
            last_output_folder = split_folders[-1]
            last_output_index = int(last_output_folder.split('_')[-1])
        else:
            last_output_index = -1
    else:
        last_output_index = -1
    output_depth_dir = outputs_dir + '/{}_{:03d}'.format(
        output_folder_prefix, last_output_index + 1)

    '''if save_output:
        if not os.path.exists(output_depth_dir):
            os.makedirs(output_depth_dir)
        else:
            raise FileExistsError('Already exists!')
        print('Output dir:', output_depth_dir)'''

    # Get images in sorted order
    images_to_use = sorted(glob.glob(input_depth_dir + '/*'))


    num_images = len(images_to_use)
    file_path2 = '../../data/live_feed/depth_completed/depth.png'
    p = Process(target = show_image, args=(file_path2,))
    for i in range(num_images):

        depth_image_path = images_to_use[i]


        # Show progress

        # Start timing

        # Load depth projections from uint16 image
        depth_image = cv2.imread(depth_image_path, cv2.IMREAD_ANYDEPTH)
        try:
            projected_depths = np.float32(depth_image / 256.0)
        except:
            return

        # Fill in
        if fill_type == 'fast':
            final_depths = depth_map_utils.fill_in_fast(
                projected_depths, extrapolate=extrapolate, blur_type=blur_type)
        elif fill_type == 'multiscale':
            final_depths, process_dict = depth_map_utils.fill_in_multiscale(
                projected_depths, extrapolate=extrapolate, blur_type=blur_type,
                show_process=show_process)

	
        else:
            raise ValueError('Invalid fill_type {}'.format(fill_type))

        # Display images from process_dict
        if fill_type == 'multiscale' and show_process:
            img_size = (570, 165)

            x_start = 80
            y_start = 50
            x_offset = img_size[0]
            y_offset = img_size[1]
            x_padding = 0
            y_padding = 28

            img_x = x_start
            img_y = y_start
            max_x = 1900

            row_idx = 0
            for key, value in process_dict.items():

                image_jet = cv2.applyColorMap(
                    np.uint8(value / np.amax(value) * 255),
                    cv2.COLORMAP_JET)
                vis_utils.cv2_show_image(
                    key, image_jet,
                    img_size, (img_x, img_y))

                img_x += x_offset + x_padding
                if (img_x + x_offset + x_padding) > max_x:
                    img_x = x_start
                    row_idx += 1
                img_y = y_start + row_idx * (y_offset + y_padding)

                # Save process images
                cv2.imwrite('process/' + key + '.png', image_jet)

            cv2.waitKey()

        # Save depth images to disk
        
        if save_depth_maps:
            depth_image_file_name = os.path.split(depth_image_path)[1]

            # Save depth map to a uint16 png (same format as disparity maps)

            
            with open(file_path2,'wb') as f2:
                depth_image = (final_depths * 256).astype(np.uint16)
                writer = png.Writer(width=depth_image.shape[1],
                                    height=depth_image.shape[0],
                                    bitdepth=16,
                                    greyscale=True)
                writer.write(f2, depth_image)
            #os.system("xdg-open " + str(file_path2))
            #p.start()
            Mat = cv2.imread(file_path2)
            cv2.imshow("Depth completed IPbasic",Mat)
            if cv2.waitKey(1) & 0xFF ==ord('q'):
                return


    print("Depth completion done")
    #p.join()


if __name__ == "__main__":
    while(1):
        main()
