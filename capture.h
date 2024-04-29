/**
 *  @file       capture.h
 *  @author     Mark Sherman
 *  @date       4/28/2024
 */

#ifndef CAPTURE_H_
#define CAPTURE_H_

int configLiveStream(void);
void *liveStream_func(void *threadp);
void stopLiveStream(void);

#endif
