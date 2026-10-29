#ifndef GX_CORE_EXPORT_H
#define GX_CORE_EXPORT_H

int ioctl_process(void* private_data, unsigned int cmd, unsigned long arg);
unsigned int poll_process(void* private_data, void *file, void *wait);
void* open_process(void* u3v_dev);
void release_process(void* private_data);
void* get_u3v(void* private_data);

#endif