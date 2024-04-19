/**
 *
 * 
 */

#include <stdio.h>
#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

static const int ESCAPE_KEY = 27;

void *liveStream_func(void *threadp) {
    VideoCapture cam0(0);
    char winInput = 0;
    namedWindow("video_display");

    if (!cam0.isOpened()){
        return NULL;
    }

    cam0.set(CAP_PROP_FRAME_WIDTH,  640);
    cam0.set(CAP_PROP_FRAME_HEIGHT, 480);

    while (1) {
        Mat frame;
        cam0.read(frame);
        imshow("video_display", frame);

        if ((winInput = waitKey(10)) == ESCAPE_KEY) {
            break;
        }
    }
    destroyWindow("video_display"); 
    return NULL;
}
