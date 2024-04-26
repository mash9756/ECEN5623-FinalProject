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

static const int ESCAPE_KEY = 27;

/* WCET timing */
static struct timespec liveStreamWCET   = {0, 0};
struct timespec liveStreamStart         = {0, 0};
struct timespec liveStreamFinish        = {0, 0};
struct timespec liveStreamDelta         = {0, 0};

static bool stopLiveStreamFlag = false;

void stopLiveStream(void) {
    stopLiveStreamFlag = true;
    printf("\n\tStopping liveStream service...");
}

void *liveStream_func(void *threadp) {
    VideoCapture cam0(0);
    char winInput = 0;
    Mat frame1;
    
    //struct timespec sleepTime = {3, 0};
    //struct timespec sleepLeft = {0, 0};

    namedWindow("video_display");

    if (!cam0.isOpened()){
        pthread_exit(NULL);
    }

    cam0.set(CAP_PROP_FRAME_WIDTH,  640);
    cam0.set(CAP_PROP_FRAME_HEIGHT, 480);
    
    for (size_t i = 0; i < 100; i++) {
        cam0.read(frame1);
        imshow("video_display", frame1);
    }

    while (!stopLiveStreamFlag) {
        clock_gettime(CLOCK_REALTIME, &liveStreamStart);
        Mat frameX;
        cam0.read(frameX);
        imshow("video_display", frameX);

        if ((winInput = waitKey(5)) == ESCAPE_KEY) {
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
    printf("\n\t\tliveStream WCET %lfms", timestamp(&liveStreamWCET));
    pthread_exit(NULL);
}
