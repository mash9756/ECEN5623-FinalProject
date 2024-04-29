/**
 *  @file       capture.cpp
 *  @author     Mark Sherman
 *  @date       4/28/2024
 */

#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>

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

/* thread exit flag, set by SIGINT handler */
static bool stopLiveStreamFlag = false;

/**
 *  @name   stopLiveStream
 *  @brief  signals program exit was triggered, break liveStream service loop and delay for final execution
 * 
 *  @param  NONE
 *  @return VOID
*/
void stopLiveStream(void) {
    stopLiveStreamFlag = true;
    printf("\tStopping liveStream service...\n");
    gpioDelay(EXIT_DELAY);
}

/**
 *  @name   liveStream_func
 *  @brief  live video stream service function
 *          running at max speed on its own core, offers ~25 FPS
 *          displays the object detection field to the user
 * 
 *  @param  threadp     thread parameters, unused
 *  @return VOID
*/
void *liveStream_func(void *threadp) {
/* open the camera for streaming */
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
    /* start of liveStream service, read and display a frame */
        clock_gettime(CLOCK_REALTIME, &liveStreamStart);
        delta_t(&liveStreamStart, getSystemStartTime(), &liveStreamStart);
        syslog(LOG_NOTICE, "\tliveStream service start %.02fms\n", timestamp(&liveStreamStart));

        cam0.read(frameX);
        imshow("video_display", frameX);

    /* close streaming window on ESC */
        if ((winInput = waitKey(5)) == ESCAPE_KEY) {
            break;
        }

    /* calculate execution time, store WCET if it occurred */
        clock_gettime(CLOCK_REALTIME, &liveStreamFinish);
        delta_t(&liveStreamFinish, getSystemStartTime(), &liveStreamFinish);
        delta_t(&liveStreamFinish, &liveStreamStart, &liveStreamDelta);
        syslog(LOG_NOTICE, "\tliveStream service end: %.02fms | ET: %.02fms\n", timestamp(&liveStreamFinish), timestamp(&liveStreamDelta));
        if(updateWCET(&liveStreamDelta, &liveStreamWCET)) {
            syslog(LOG_NOTICE, "\tliveStream WCET Updated: %.02fms\n", timestamp(&liveStreamWCET));
        }
    }
/* thread exit cleanup */
    cam0.release();
    destroyWindow("video_display"); 
    printf("\t\tFinal liveStream WCET %.02fms\n", timestamp(&liveStreamWCET));
    syslog(LOG_NOTICE, "\t\tFinal liveStream WCET %.02fms\n", timestamp(&liveStreamWCET));
    pthread_exit(NULL);
}
