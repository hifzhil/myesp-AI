#include "esp_log.h"
#include "dl_image.hpp"
#include "dl_detect_define.hpp"
#include <list>
#include <vector>

std::pair<int, int> get_mouth_center(std::list<dl::detect::result_t> &results);

        /**
         * @brief Crop a patch from image and resize and store to destination image.
         * If the cropping box is out of image, destination image will be padded with edge.
         * 
         * The outer rectangle is the entire output image.
         * The inner rectangle is where the resized image will be stored.
         * In other world, this function could help you do padding while resize image.
         *               ___________________________(dst_w)__________________
         *              |         ___________________________                |
         *              |        |(x_start, y_start)         |               | 
         *              |        |                           |               | 
         *              |        |                           |               | 
         *       (dst_h)|        |                           |               | 
         *              |        |                           |               | 
         *              |        |                           |               | 
         *              |        |___________________________|(x_end, y_end) | 
         *              |____________________________________________________| 
         * 
         * @tparam T suppot all integer types
         * @param dst_image     pointer of destination(output) image
         * @param dst_width     destination image width
         * @param dst_channel   destination image channel number
         * @param dst_y_start   start y of resized image in destination image
         * @param dst_y_end     end y of resized image in destination image
         * @param dst_x_start   start x of resized image in destination image
         * @param dst_x_end     end x of resized image in destination image
         * @param src_image     pointer of source image
         * @param src_height    source image height
         * @param src_width     source image width
         * @param src_channel   source image channel
         * @param src_y_start   start y of resized image in source image
         * @param src_y_end     end y of resized image in source image
         * @param src_x_start   start x of resized image in source image
         * @param src_x_end     end x of resized image in source image
         * @param resize_type   one of IMAGE_RESIZE_BILINEAR or IMAGE_RESIZE_MEAN or IMAGE_RESIZE_NEAREST
         * @param shift_left    bit left shift number implemented on output
         */