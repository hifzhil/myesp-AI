/*
  * ESPRESSIF MIT License
  *
  * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
  *
  * Permission is hereby granted for use on ESPRESSIF SYSTEMS products only, in which case,
  * it is free of charge, to any person obtaining a copy of this software and associated
  * documentation files (the "Software"), to deal in the Software without restriction, including
  * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
  * to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all copies or
  * substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
  * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
  * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
  * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
  * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  *
  */
#pragma once
#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>
#include <math.h>
// #include "mtmn.h"
#include "esp_camera.h"

#define LANDMARKS_NUM (10)

#define MAX_VALID_COUNT_PER_IMAGE (30)

#define DL_IMAGE_MIN(A, B) ((A) < (B) ? (A) : (B))
#define DL_IMAGE_MAX(A, B) ((A) < (B) ? (B) : (A))

#define RGB565_MASK_RED 0xF800
#define RGB565_MASK_GREEN 0x07E0
#define RGB565_MASK_BLUE 0x001F

    typedef enum
    {
        BINARY, /*!< binary */
    } en_threshold_mode;


    camera_fb_t* process_frame(camera_fb_t *frame);
    /**
     * @brief Convert RGB565 image to RGB888 image
     * 
     * @param in    Input RGB565 image
     * @param dst   Resulting RGB888 image
     */
    static inline void rgb565_to_888(uint16_t in, uint8_t *dst)
    { /*{{{*/
        in = (in & 0xFF) << 8 | (in & 0xFF00) >> 8;
        dst[2] = (in & RGB565_MASK_BLUE) << 3;  // blue
        dst[1] = (in & RGB565_MASK_GREEN) >> 3; // green
        dst[0] = (in & RGB565_MASK_RED) >> 8;   // red

        // dst[0] = (in & 0x1F00) >> 5;
        // dst[1] = ((in & 0x7) << 5) | ((in & 0xE000) >> 11);
        // dst[2] = in & 0xF8;
    } /*}}}*/

    static inline void rgb565_to_888_q16(uint16_t in, int16_t *dst)
    { /*{{{*/
        in = (in & 0xFF) << 8 | (in & 0xFF00) >> 8;
        dst[2] = (in & RGB565_MASK_BLUE) << 3;  // blue
        dst[1] = (in & RGB565_MASK_GREEN) >> 3; // green
        dst[0] = (in & RGB565_MASK_RED) >> 8;   // red

        // dst[0] = (in & 0x1F00) >> 5;
        // dst[1] = ((in & 0x7) << 5) | ((in & 0xE000) >> 11);
        // dst[2] = in & 0xF8;
    } /*}}}*/
    /**@}*/

    /**
     * @brief Convert RGB888 image to RGB565 image
     * 
     * @param in      Resulting RGB565 image
     * @param r       The red channel of the Input RGB888 image 
     * @param g       The green channel of the Input RGB888 image 
     * @param b       The blue channel of the Input RGB888 image
     */
    static inline void rgb888_to_565(uint16_t *in, uint8_t r, uint8_t g, uint8_t b)
    { /*{{{*/
        uint16_t rgb565 = 0;
        rgb565 = ((r >> 3) << 11);
        rgb565 |= ((g >> 2) << 5);
        rgb565 |= (b >> 3);
        rgb565 = (rgb565 & 0xFF) << 8 | (rgb565 & 0xFF00) >> 8;
        *in = rgb565;
    } /*}}}*/

    /**
     * @brief Filter out the resulting boxes whose confidence score is lower than the threshold and convert the boxes to the actual boxes on the original image.((x, y, w, h) -> (x1, y1, x2, y2))
     * 
     * @param score                    Confidence score of the boxes
     * @param offset                   The predicted anchor-based offset
     * @param landmark                 The landmarks corresponding to the box
     * @param width                    Height of the original image
     * @param height                   Width of the original image
     * @param anchor_number            Anchor number of the detection output feature map 
     * @param anchors_size             The anchor size
     * @param score_threshold          Threshold of the confidence score
     * @param stride 
     * @param resized_height_scale 
     * @param resized_width_scale 
     * @param do_regression 
     * @return image_list_t* 
     */
    // image_list_t *image_get_valid_boxes(fptp_t *score,
    //                                     fptp_t *offset,
    //                                     fptp_t *landmark,
    //                                     int width,
    //                                     int height,
    //                                     int anchor_number,
    //                                     int *anchors_size,
    //                                     fptp_t score_threshold,
    //                                     int stride,
    //                                     fptp_t resized_height_scale,
    //                                     fptp_t resized_width_scale,
    //                                     bool do_regression);
    // /**
    //  * @brief Sort the resulting box lists by their confidence score.
    //  * 
    //  * @param image_sorted_list     The sorted box list.
    //  * @param insert_list           The box list that have not been sorted.
    //  */
    // void image_sort_insert_by_score(image_list_t *image_sorted_list, const image_list_t *insert_list);

    // /**
    //  * @brief Run NMS algorithm 
    //  * 
    //  * @param image_list         The input boxes list
    //  * @param nms_threshold      NMS threshold
    //  * @param same_area          The flag of boxes with same area
    //  */
    // void image_nms_process(image_list_t *image_list, fptp_t nms_threshold, int same_area);

    // /**
    //  * @brief Resize an image to half size 
    //  * 
    //  * @param dimage      The output image
    //  * @param dw          Width of the output image
    //  * @param dh          Height of the output image
    //  * @param dc          Channel of the output image
    //  * @param simage      Source image
    //  * @param sw          Width of the source image
    //  * @param sc          Channel of the source image
    //  */
    void image_zoom_in_twice(uint8_t *dimage,
                             int dw,
                             int dh,
                             int dc,
                             uint8_t *simage,
                             int sw,
                             int sc);

    /**
     * @brief Resize the image in RGB888 format via bilinear interpolation
     * 
     * @param dst_image    The output image
     * @param src_image    Source image
     * @param dst_w        Width of the output image
     * @param dst_h        Height of the output image
     * @param dst_c        Channel of the output image
     * @param src_w        Width of the source image
     * @param src_h        Height of the source image
     */
    void image_resize_linear(uint8_t *dst_image, uint8_t *src_image, int dst_w, int dst_h, int dst_c, int src_w, int src_h);

    /**
     * @brief Crop， rotate and zoom the image in RGB888 format, 
     * 
     * @param corp_image       The output image
     * @param src_image        Source image
     * @param rotate_angle     Rotate angle
     * @param ratio            scaling ratio
     * @param center           Center of rotation
     */
    void image_cropper(uint8_t *corp_image, uint8_t *src_image, int dst_w, int dst_h, int dst_c, int src_w, int src_h, float rotate_angle, float ratio, float *center);

    /**
     * @brief Convert the rgb565 image to the rgb888 image   
     * 
     * @param m       The output rgb888 image
     * @param bmp     The input rgb565 image
     * @param count   Total pixels of the rgb565 image
     */
    void image_rgb565_to_888(uint8_t *m, uint16_t *bmp, int count);

    /**
     * @brief Convert the rgb888 image to the rgb565 image
     * 
     * @param bmp     The output rgb565 image
     * @param m       The input rgb888 image
     * @param count   Total pixels of the rgb565 image
     */
    void image_rgb888_to_565(uint16_t *bmp, uint8_t *m, int count);

    /**
     * @brief draw rectangle on the rgb565 image
     * 
     * @param buf     Input image
     * @param boxes   Rectangle Boxes
     * @param width   Width of the input image
     */
    // void draw_rectangle_rgb565(uint16_t *buf, box_array_t *boxes, int width);

    // /**
    //  * @brief draw rectangle on the rgb888 image
    //  * 
    //  * @param buf     Input image
    //  * @param boxes   Rectangle Boxes
    //  * @param width   Width of the input image
    //  */
    // void draw_rectangle_rgb888(uint8_t *buf, box_array_t *boxes, int width);

    // /**
    //  * @brief Get the pixel difference of two images
    //  * 
    //  * @param dst       The output pixel difference
    //  * @param src1      Input image 1
    //  * @param src2      Input image 2
    //  * @param count     Total pixels of the input image
    //  */
    void image_abs_diff(uint8_t *dst, uint8_t *src1, uint8_t *src2, int count);

    /**
     * @brief Binarize an image to 0 and value. 
     * 
     * @param dst           The output image
     * @param src           Source image
     * @param threshold     Threshold of binarization
     * @param value         The value of binarization
     * @param count         Total pixels of the input image
     * @param mode          Threshold mode
     */
    void image_threshold(uint8_t *dst, uint8_t *src, int threshold, int value, int count, en_threshold_mode mode);

    /**
     * @brief Erode the image
     * 
     * @param dst          The output image
     * @param src          Source image
     * @param src_w        Width of the source image
     * @param src_h        Height of the source image
     * @param src_c        Channel of the source image
     */
    void image_erode(uint8_t *dst, uint8_t *src, int src_w, int src_h, int src_c);

    typedef float matrixType;
    typedef struct
    {
        int w;              /*!< width */
        int h;              /*!< height */
        matrixType **array; /*!< array */
    } Matrix;

    /**
     * @brief Allocate a 2d matrix
     * 
     * @param h                Height of matrix
     * @param w                Width of matrix
     * @return Matrix*         2d matrix
     */
    Matrix *matrix_alloc(int h, int w);

    /**
     * @brief Free a 2d matrix
     * 
     * @param m    2d matrix 
     */
    void matrix_free(Matrix *m);

    /**
     * @brief Get the similarity matrix of similarity transformation
     * 
     * @param srcx          Source x coordinates
     * @param srcy          Source y coordinates
     * @param dstx          Destination x coordinates
     * @param dsty          Destination y coordinates
     * @param num           The number of the coordinates
     * @return Matrix*      The resulting transformation matrix
     */
    Matrix *get_similarity_matrix(float *srcx, float *srcy, float *dstx, float *dsty, int num);

    /**
     * @brief Get the affine transformation matrix
     * 
     * @param srcx          Source x coordinates
     * @param srcy          Source y coordinates
     * @param dstx          Destination x coordinates
     * @param dsty          Destination y coordinates
     * @return Matrix*      The resulting transformation matrix
     */
    Matrix *get_affine_transform(float *srcx, float *srcy, float *dstx, float *dsty);

    /**
     * @brief Applies an affine transformation to an image
     * 
     * @param img           Input image
     * @param crop          Dst output image that has the size dsize and the same type as src
     * @param M             Affine transformation matrix
     */
    // void warp_affine(dl_matrix3du_t *img, dl_matrix3du_t *crop, Matrix *M);

    // /**
    //  * @brief Resize the image in RGB888 format via bilinear interpolation, and quantify the output image
    //  * 
    //  * @param dst_image            Quantized output image
    //  * @param src_image            Input image 
    //  * @param dst_w                Width of the output image 
    //  * @param dst_h                Height of the output image 
    //  * @param dst_c                Channel of the output image
    //  * @param src_w                Width of the input image 
    //  * @param src_h                Height of the input image
    //  * @param shift                Shift parameter of quantization.
    //  */
    // void image_resize_linear_q(qtp_t *dst_image, uint8_t *src_image, int dst_w, int dst_h, int dst_c, int src_w, int src_h, int shift);

    // /**
    //  * @brief Preprocess the input image of object detection model. The process is like this: resize -> normalize -> quantify
    //  * 
    //  * @param image                 Input image, RGB888 format.
    //  * @param input_w               Width of the input image.
    //  * @param input_h               Height of the input image.
    //  * @param target_size           Target size of the model input image.
    //  * @param exponent              Exponent of the quantized model input image.
    //  * @param process_mode          Process mode. 0: resize with padding to keep height == width. 1: resize without padding, height != width.  
    //  * @return dl_matrix3dq_t*      The resulting preprocessed image.
    //  */
    // dl_matrix3dq_t *image_resize_normalize_quantize(uint8_t *image, int input_w, int input_h, int target_size, int exponent, int process_mode);

    // /**
    //  * @brief Resize the image in RGB565 format via mean neighbour interpolation, and quantify the output image
    //  * 
    //  * @param dimage            Quantized output image. 
    //  * @param simage            Input image.  
    //  * @param dw                Width of the allocated output image memory.
    //  * @param dc                Channel of the allocated output image memory.
    //  * @param sw                Width of the input image. 
    //  * @param sh                Height of the input image. 
    //  * @param tw                Target width of the output image.
    //  * @param th                Target height of the output image.
    //  * @param shift             Shift parameter of quantization.
    //  */
    // void image_resize_shift_fast(qtp_t *dimage, uint16_t *simage, int dw, int dc, int sw, int sh, int tw, int th, int shift);

    // /**
    //  * @brief Resize the image in RGB565 format via nearest neighbour interpolation, and quantify the output image
    //  * 
    //  * @param dimage            Quantized output image. 
    //  * @param simage            Input image.  
    //  * @param dw                Width of the allocated output image memory.
    //  * @param dc                Channel of the allocated output image memory.
    //  * @param sw                Width of the input image. 
    //  * @param sh                Height of the input image. 
    //  * @param tw                Target width of the output image.
    //  * @param th                Target height of the output image.
    //  * @param shift             Shift parameter of quantization.
    //  */
    // void image_resize_nearest_shift(qtp_t *dimage, uint16_t *simage, int dw, int dc, int sw, int sh, int tw, int th, int shift);

    // /**
    //  * @brief Crop the image in RGB565 format and resize it to target size, then quantify the output image 
    //  * 
    //  * @param dimage            Quantized output image. 
    //  * @param simage            Input image.
    //  * @param dw                Target size of the output image.
    //  * @param sw                Width of the input image. 
    //  * @param sh                Height of the input image. 
    //  * @param x1                The x coordinate of the upper left corner of the cropped area
    //  * @param y1                The y coordinate of the upper left corner of the cropped area
    //  * @param x2                The x coordinate of the lower right corner of the cropped area
    //  * @param y2                The y coordinate of the lower right corner of the cropped area
    //  * @param shift             Shift parameter of quantization.
    //  */
    // void image_crop_shift_fast(qtp_t *dimage, uint16_t *simage, int dw, int sw, int sh, int x1, int y1, int x2, int y2, int shift);

#ifdef __cplusplus
}
#endif
