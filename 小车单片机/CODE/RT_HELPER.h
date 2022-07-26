#ifndef __RT_HELPER_h__
#define __RT_HELPER_h__
#define EVENT_FLAG1     (1<<1)
#define EVENT_FLAG2     (1<<2)
#define EVENT_FLAG3     (1<<3)
#define EVENT_FLAG4     (1<<4)
#define EVENT_FLAG5     (1<<5)
#define EVENT_FLAG6     (1<<6)
#define EVENT_FLAG7     (1<<7)
#define EVENT_FLAG8     (1<<8)
#define EVENT_FLAG9     (1<<9)
#define EVENT_FLAG10    (1<<10)
#define EVENT_FLAG11    (1<<11)
#define EVENT_FLAG12    (1<<12)
#define EVENT_FLAG13    (1<<13)
#define EVENT_FLAG14    (1<<14)
#define EVENT_FLAG15    (1<<15)

extern rt_event_t event;
extern rt_sem_t key_sem;
extern rt_sem_t dma_sem;
extern int print_flag;
extern int start_thread[20];
extern int end_thread[20];

#endif
