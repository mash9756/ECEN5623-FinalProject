/**
 *
 * 
 */

#include <stdio.h>
#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pigpio.h>

#include "misc.h"

using namespace cv;

constexpr int ESCAPE_KEY = 27;

/* WCET timing */
static struct timespec liveStreamWCET   = {0, 0};
struct timespec liveStreamStart         = {0, 0};
struct timespec liveStreamFinish        = {0, 0};
struct timespec liveStreamDelta         = {0, 0};

static bool stopLiveStreamFlag = false;

void stopLiveStream(void) {
    stopLiveStreamFlag = true;
    printf("\tStopping liveStream service...\n");
    gpioDelay(1000000);
}

void *liveStream_func(void *threadp) {
    VideoCapture cam0(0);
    char winInput = 0;
    Mat frameX;

    if (!cam0.isOpened()) {
        printf("Can't open cam0!\n");
        pthread_exit(NULL);
    }

/* setup display window and camera resolution */
    namedWindow("video_display", (WINDOW_NORMAL && WINDOW_KEEPRATIO));
    cam0.set(CAP_PROP_FRAME_WIDTH,  320);
    cam0.set(CAP_PROP_FRAME_HEIGHT, 240);
    cam0.set(CAP_PROP_FPS, 30);
    
    while (!stopLiveStreamFlag) {
        clock_gettime(CLOCK_REALTIME, &liveStreamStart);
        cam0.read(frameX);
        imshow("video_display", frameX);

    /* close streaming window on ESC */
        if ((winInput = waitKey(5)) == ESCAPE_KEY) {
            break;
        }
        
        clock_gettime(CLOCK_REALTIME, &liveStreamFinish);
        delta_t(&liveStreamFinish, &liveStreamStart, &liveStreamDelta);
        if(timestamp(&liveStreamDelta) > timestamp(&liveStreamWCET)) {
            liveStreamWCET.tv_sec    = liveStreamDelta.tv_sec;
            liveStreamWCET.tv_nsec   = liveStreamDelta.tv_nsec;
            //printf("\tliveStream WCET %lfms\n", timestamp(&liveStreamWCET));
            //syslog(LOG_NOTICE, "\talarm WCET %lf msec\n", timestamp(&WCET));
        }
    }
    cam0.release();
    destroyWindow("video_display"); 
    printf("\t\tFinal liveStream WCET %lfms\n", timestamp(&liveStreamWCET));
    pthread_exit(NULL);
}
