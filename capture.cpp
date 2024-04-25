/**
 *
 * 
 */

#include <stdio.h>
#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "misc.h"

using namespace cv;

static const int ESCAPE_KEY = 27;

/* WCET timing */
static struct timespec liveStreamWCET   = {0, 0};
struct timespec liveStreamStart         = {0, 0};
struct timespec liveStreamFinish        = {0, 0};
struct timespec liveStreamDelta         = {0, 0};

void *liveStream_func(void *threadp) {
    VideoCapture cam0(0);
    char winInput = 0;
    namedWindow("video_display");

    if (!cam0.isOpened()){
        pthread_exit(NULL);
    }

    cam0.set(CAP_PROP_FRAME_WIDTH,  640);
    cam0.set(CAP_PROP_FRAME_HEIGHT, 480);

    while (1) {
        clock_gettime(CLOCK_REALTIME, &liveStreamStart);

        Mat frame;
        cam0.read(frame);
        imshow("video_display", frame);

        if ((winInput = waitKey(10)) == ESCAPE_KEY) {
            break;
        }
        
        clock_gettime(CLOCK_REALTIME, &liveStreamFinish);
        delta_t(&liveStreamFinish, &liveStreamStart, &liveStreamDelta);
        if(timestamp(&liveStreamDelta) > timestamp(&liveStreamWCET)) {
            liveStreamWCET.tv_sec    = liveStreamDelta.tv_sec;
            liveStreamWCET.tv_nsec   = liveStreamDelta.tv_nsec;
            printf("\tliveStream WCET %lfms\n", timestamp(&liveStreamWCET));
            //syslog(LOG_NOTICE, "\talarm WCET %lf msec\n", timestamp(&WCET));
        }
    }
    destroyWindow("video_display"); 
    pthread_exit(NULL);
}
