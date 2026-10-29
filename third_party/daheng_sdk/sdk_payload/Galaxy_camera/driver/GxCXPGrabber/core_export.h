#ifndef GX_CORE_EXPORT_H
#define GX_CORE_EXPORT_H

// #include "common.h"

unsigned int create_card_device(void **p, void* pci_dev);
void destroy_card_device(void *p);
unsigned int init_card_device(void *p);
void uninit_card_device(void *p);
unsigned int resume_card(void* p);

int open_process(char* file_name, void* card_device_private);
int release_process(char* file_name, void* card_device_private);
int ioctl_process(char* file_name, void* card_device_private, 
                        unsigned int cmd, unsigned long arg);
unsigned int poll_process(char* file_name, void* card_device_private, 
                        void* file, void* wait);

#endif