/**
 * 
 *
 *  
*/

#ifndef CAPTURE_H_
#define CAPTURE_H_

int configLiveStream(void);
void *liveStream_func(void *threadp);
void stopLiveStream(void);

#endif
