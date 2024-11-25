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
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "image_util.h"
#include "esp_timer.h"

void image_zoom_in_twice(uint8_t *dimage,
                         int dw,
                         int dh,
                         int dc,
                         uint8_t *simage,
                         int sw,
                         int sc)
{
    for (int dyi = 0; dyi < dh; dyi++)
    {
        int _di = dyi * dw;

        int _si0 = dyi * 2 * sw;
        int _si1 = _si0 + sw;

        for (int dxi = 0; dxi < dw; dxi++)
        {
            int di = (_di + dxi) * dc;
            int si0 = (_si0 + dxi * 2) * sc;
            int si1 = (_si1 + dxi * 2) * sc;

            if (1 == dc)
            {
                dimage[di] = (uint8_t)((simage[si0] + simage[si0 + 1] + simage[si1] + simage[si1 + 1]) >> 2);
            }
            else if (3 == dc)
            {
                dimage[di] = (uint8_t)((simage[si0] + simage[si0 + 3] + simage[si1] + simage[si1 + 3]) >> 2);
                dimage[di + 1] = (uint8_t)((simage[si0 + 1] + simage[si0 + 4] + simage[si1 + 1] + simage[si1 + 4]) >> 2);
                dimage[di + 2] = (uint8_t)((simage[si0 + 2] + simage[si0 + 5] + simage[si1 + 2] + simage[si1 + 5]) >> 2);
            }
            else
            {
                for (int dci = 0; dci < dc; dci++)
                {
                    dimage[di + dci] = (uint8_t)((simage[si0 + dci] + simage[si0 + 3 + dci] + simage[si1 + dci] + simage[si1 + 3 + dci] + 2) >> 2);
                }
            }
        }
    }
    return;
}

void image_resize_linear(uint8_t *dst_image, uint8_t *src_image, int dst_w, int dst_h, int dst_c, int src_w, int src_h)
{ /*{{{*/
    float scale_x = (float)src_w / dst_w;
    float scale_y = (float)src_h / dst_h;

    int dst_stride = dst_c * dst_w;
    int src_stride = dst_c * src_w;

    if (fabs(scale_x - 2) <= 1e-6 && fabs(scale_y - 2) <= 1e-6)
    {
        image_zoom_in_twice(
            dst_image,
            dst_w,
            dst_h,
            dst_c,
            src_image,
            src_w,
            dst_c);
    }
    else
    {
        for (int y = 0; y < dst_h; y++)
        {
            float fy[2];
            fy[0] = (float)((y + 0.5) * scale_y - 0.5); // y
            int src_y = (int)fy[0];                     // y1
            fy[0] -= src_y;                             // y - y1
            fy[1] = 1 - fy[0];                          // y2 - y
            src_y = DL_IMAGE_MAX(0, src_y);
            src_y = DL_IMAGE_MIN(src_y, src_h - 2);

            for (int x = 0; x < dst_w; x++)
            {
                float fx[2];
                fx[0] = (float)((x + 0.5) * scale_x - 0.5); // x
                int src_x = (int)fx[0];                     // x1
                fx[0] -= src_x;                             // x - x1
                if (src_x < 0)
                {
                    fx[0] = 0;
                    src_x = 0;
                }
                if (src_x > src_w - 2)
                {
                    fx[0] = 0;
                    src_x = src_w - 2;
                }
                fx[1] = 1 - fx[0]; // x2 - x

                for (int c = 0; c < dst_c; c++)
                {
                    dst_image[y * dst_stride + x * dst_c + c] = round(src_image[src_y * src_stride + src_x * dst_c + c] * fx[1] * fy[1] + src_image[src_y * src_stride + (src_x + 1) * dst_c + c] * fx[0] * fy[1] + src_image[(src_y + 1) * src_stride + src_x * dst_c + c] * fx[1] * fy[0] + src_image[(src_y + 1) * src_stride + (src_x + 1) * dst_c + c] * fx[0] * fy[0]);
                }
            }
        }
    }
} /*}}}*/

void image_cropper(uint8_t *rot_data, uint8_t *src_data, int rot_w, int rot_h, int rot_c, int src_w, int src_h, float rotate_angle, float ratio, float *center)
{ /*{{{*/
    int rot_stride = rot_w * rot_c;
    float rot_w_start = 0.5f - (float)rot_w / 2;
    float rot_h_start = 0.5f - (float)rot_h / 2;

    //rotate_angle must be radius
    float si = sin(rotate_angle);
    float co = cos(rotate_angle);

    int src_stride = src_w * rot_c;

    for (int y = 0; y < rot_h; y++)
    {
        for (int x = 0; x < rot_w; x++)
        {
            float xs, ys, xr, yr;
            xs = ratio * (rot_w_start + x);
            ys = ratio * (rot_h_start + y);

            xr = xs * co + ys * si;
            yr = -xs * si + ys * co;

            float fy[2];
            fy[0] = center[1] + yr; // y
            int src_y = (int)fy[0]; // y1
            fy[0] -= src_y;         // y - y1
            fy[1] = 1 - fy[0];      // y2 - y
            src_y = DL_IMAGE_MAX(0, src_y);
            src_y = DL_IMAGE_MIN(src_y, src_h - 2);

            float fx[2];
            fx[0] = center[0] + xr; // x
            int src_x = (int)fx[0]; // x1
            fx[0] -= src_x;         // x - x1
            if (src_x < 0)
            {
                fx[0] = 0;
                src_x = 0;
            }
            if (src_x > src_w - 2)
            {
                fx[0] = 0;
                src_x = src_w - 2;
            }
            fx[1] = 1 - fx[0]; // x2 - x

            for (int c = 0; c < rot_c; c++)
            {
                rot_data[y * rot_stride + x * rot_c + c] = round(src_data[src_y * src_stride + src_x * rot_c + c] * fx[1] * fy[1] + src_data[src_y * src_stride + (src_x + 1) * rot_c + c] * fx[0] * fy[1] + src_data[(src_y + 1) * src_stride + src_x * rot_c + c] * fx[1] * fy[0] + src_data[(src_y + 1) * src_stride + (src_x + 1) * rot_c + c] * fx[0] * fy[0]);
            }
        }
    }
} /*}}}*/