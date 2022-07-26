#include "headfile.h"
#include "RT_HELPER.h"




rt_event_t event;
rt_sem_t key_sem = RT_NULL;
rt_sem_t dma_sem = RT_NULL;
rt_mq_t message_queue;
int print_flag=0;
int start_thread[20];
int end_thread[20];
int num_thread[20];

