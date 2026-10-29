#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/usb.h>
#include "common.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 8, 0)
#define mmap_read_lock(mm) down_read(&mm->mmap_sem)
#define mmap_read_unlock(mm) up_read(&mm->mmap_sem)
#else
#include <linux/mmap_lock.h>
#endif

#define TRACE_BUF_SIZE 512
typedef int (*iterate_shared)(struct file *, struct dir_context *);
typedef int (*thread_func)(void* context);

void gx_dump_stack(void)
{
    dump_stack();
}

void gx_cond_resched(void)
{
    cond_resched();
}

int gx_current_pid(void)
{
    return current->pid;
}

void *gx_current_comm(void)
{
    return current->active_mm;
}

unsigned int gx_gfp_kernel(void)
{
    return GFP_KERNEL;
}

unsigned int gx_gfp_atomic(void)
{
    return GFP_ATOMIC;
}

void *gx_kmalloc(size_t size, unsigned int flags)
{
    return kmalloc(size, flags);
}

void *gx_kzalloc(size_t size, unsigned int flags)
{
    return kzalloc(size, flags);
}

void *gx_kcalloc(size_t n, size_t size, unsigned int flags)
{
    return kcalloc(n, size, flags);
}

void gx_kfree(void *p)
{
    kfree(p);
}

void *gx_vmalloc(size_t size)
{
    return vmalloc(size);
}

void gx_vfree(void *p)
{
    vfree(p);
}

void gx_memset(void *s, char c, size_t count)
{
    memset(s, c, count);
}

void gx_memcpy(void *to, const void *from, size_t n)
{
    memcpy(to, from, n);
}

size_t gx_strlen(const char *s)
{
    return strlen(s);
}

char *gx_strstr(const char *cs, const char *ct)
{
    return strstr(cs, ct);
}

char *gx_strncpy(char *dest, const char *src, size_t count)
{
    return strncpy(dest, src, count);
}

long gx_simple_strtol(const char *cp, char **endp, unsigned int base)
{
    return simple_strtol(cp, endp, base);
}

unsigned long gx_copy_from_user(void *to, unsigned long from, unsigned long n)
{
    const void __user *uaddr = (const void __user *)(uintptr_t)from;
    return copy_from_user(to, uaddr, n);
}

unsigned long gx_copy_to_user(unsigned long to, const void *from, unsigned long n)
{
    const void __user *uaddr = (const void __user *)(uintptr_t)to;
    return copy_to_user(uaddr, from, n);
}

int gx_get_user_pages_fast(unsigned long start, int nr_pages,
                           unsigned int gup_flags, void **pages)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)
    return pin_user_pages_fast(start, nr_pages, gup_flags, (struct page **)pages);
#else
    int i = 0;
    int ret = get_user_pages_fast(start, nr_pages, gup_flags, (struct page **)pages);
    if (ret != nr_pages)
    {
        return ret;
    }

    for (i = 0; i < nr_pages; i++)
    {
        SetPageReserved(((struct page **)pages)[i]);
    }

    return ret;
#endif
}

long gx_get_user_pages(unsigned long start, unsigned long nr_pages,
                       unsigned int gup_flags, void **pages, void **vmas)
{
    long ret = 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
    mmap_read_lock(current->mm);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 5, 0)
    ret = pin_user_pages(start, nr_pages, gup_flags, (struct page **)pages);
#else
    ret = pin_user_pages(start, nr_pages, gup_flags, (struct page **)pages, 
                         (struct vm_area_struct **)vmas);
#endif
    mmap_read_unlock(current->mm);
    return ret;
#else
    int i = 0;
    mmap_read_lock(current->mm);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 5, 0)
    ret = get_user_pages(start, nr_pages, gup_flags, (struct page **)pages);
#else
    ret = get_user_pages(start, nr_pages, gup_flags, (struct page **)pages, 
                         (struct vm_area_struct **)vmas);
#endif
    mmap_read_unlock(current->mm);

    if (ret != nr_pages)
    {
        return ret;
    }

    for (i = 0; i < nr_pages; i++)
    {
        SetPageReserved(((struct page **)pages)[i]);
    }

    return ret;
#endif
}

void gx_put_user_pages(void **pages, unsigned long npages)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
    unpin_user_pages((struct page **)pages, npages);
#else
    int i = 0;
    for (i = 0; i < npages; i++)
    {
        ClearPageReserved(((struct page **)pages)[i]);
        put_page(((struct page **)pages)[i]);
    }
#endif
}

void *gx_vmalloc_to_page(const void *addr)
{
    return vmalloc_to_page(addr);
}

unsigned long gx_page_to_phys(void *page)
{
    return page_to_phys((struct page *)page);
}

unsigned long long gx_virt_to_phys(volatile void *address)
{
    return virt_to_phys(address);
}

int gx_dma_mapping_error(void *dev, unsigned long long dma_addr)
{
    return dma_mapping_error(dev, dma_addr);
}

unsigned long long gx_dma_map_page(void *dev, void *page, size_t offset,
                                   size_t size, enum dma_data_direction dir)
{
    return dma_map_page(dev, page, offset, size, dir);
}

void gx_dma_unmap_page(void *dev, unsigned long long addr, size_t size,
                       enum dma_data_direction dir)
{
    dma_unmap_page(dev, addr, size, dir);
}

void *gx_dma_alloc_coherent(void *dev, size_t size, unsigned long long *dma_handle)
{
    return dma_alloc_coherent(dev, size, dma_handle, GFP_KERNEL);
}

void gx_dma_free_coherent(void *dev, size_t size,
                          void *cpu_addr, unsigned long long dma_handle)
{
    dma_free_coherent(dev, size, cpu_addr, dma_handle);
}

void gx_dma_sync_single_for_cpu(void *dev, unsigned long long addr,
                                size_t size, enum dma_data_direction dir)
{
    dma_sync_single_for_cpu(dev, addr, size, dir);
}

void gx_vunmap(void *addr)
{
    vunmap(addr);
}

void *gx_get_device(void *pci_dev)
{
    struct pci_dev *pdev = (struct pci_dev *)pci_dev;
    return &pdev->dev;
}

unsigned int gx_get_pci_bus_num(void *pci_dev)
{
    struct pci_dev *pdev = (struct pci_dev *)pci_dev;
    return pdev->bus->number;
}

unsigned int gx_get_pci_dev_num(void *pci_dev)
{
    struct pci_dev *pdev = (struct pci_dev *)pci_dev;
    return PCI_SLOT(pdev->devfn);
}

unsigned int gx_get_pci_fun_num(void *pci_dev)
{
    struct pci_dev *pdev = (struct pci_dev *)pci_dev;
    return PCI_FUNC(pdev->devfn);
}

void* gx_get_pci_dev(void *pci_dev)
{
    struct pci_dev *pdev = (struct pci_dev *)pci_dev;
    return &pdev->dev;
}

int gx_pci_find_capability(void *dev, int cap)
{
    return pci_find_capability(dev, cap);
}

int gx_get_pci_msi_enabled(void *pci_dev)
{
    struct pci_dev *pdev = (struct pci_dev *)pci_dev;
    return pdev->msi_enabled;
}

int gx_get_pci_msix_enabled(void *pci_dev)
{
    struct pci_dev *pdev = (struct pci_dev *)pci_dev;
    return pdev->msix_enabled;
}

unsigned int gx_get_pci_irq(void *pci_dev)
{
    struct pci_dev *pdev = (struct pci_dev *)pci_dev;
    return pdev->irq;
}

unsigned long gx_pci_resource_len(void *pci_dev, int index)
{
    struct pci_dev *pdev = (struct pci_dev *)pci_dev;
    return pci_resource_len(pdev, index);
}

void *gx_create_spin_lock(void)
{
    return gx_kzalloc(sizeof(spinlock_t), GFP_KERNEL);
}

void gx_spin_lock_init(void *lock)
{
    spin_lock_init((spinlock_t *)lock);
}

void gx_spin_lock_irqsave(void *lock, unsigned long *flags)
{
    spin_lock_irqsave(lock, *flags);
}

void gx_spin_unlock_irqrestore(void *lock, unsigned long *flags)
{
    spin_unlock_irqrestore(lock, *flags);
}

void *gx_create_tasklet(void)
{
    return gx_kzalloc(sizeof(struct tasklet_struct), GFP_KERNEL);
}

void gx_tasklet_init(void *t, void (*func)(unsigned long),
                     unsigned long data)
{
    tasklet_init(t, func, data);
}

void gx_tasklet_schedule(void *t)
{
    tasklet_schedule(t);
}

void *gx_create_completion(void)
{
    return gx_kzalloc(sizeof(struct completion), GFP_KERNEL);
}

void gx_init_completion(void *t)
{
    init_completion(t);
}

void gx_complete(void *t)
{
    complete(t);
}

void gx_wait_for_completion_timeout(void *t, u64 timeout)
{
    wait_for_completion_timeout(t, msecs_to_jiffies(timeout));
}

void* gx_kthread_run(thread_func func, void* data, char* name)
{
    return (void*)kthread_run(func, data, name);
}

void gx_kthread_stop(void* t)
{
    kthread_stop((struct task_struct *)t);
}

bool gx_kthread_should_stop(void)
{
    return kthread_should_stop();
}

void gx_poll_wait(void *filp, void *wait_address, void *p)
{
    poll_wait(filp, wait_address, p);
}

void gx_wake_up_interruptible(void *wq_head)
{
    wake_up_interruptible(wq_head);
}

typedef bool (*condition_fn)(void *);
int gx_wait_event_interruptible_timeout(void *wq_head, condition_fn fn, void *data, u32 timeout)
{
    return wait_event_interruptible_timeout(
        *(struct wait_queue_head *)wq_head,
        fn(data),
        msecs_to_jiffies(timeout)
    );
}

void *gx_create_wait_queue_head(void)
{
    return gx_kzalloc(sizeof(struct wait_queue_head), GFP_KERNEL);
}

void gx_init_waitqueue_head(void *wq_head)
{
    init_waitqueue_head(wq_head);
}

unsigned int gx_swab32(unsigned int data)
{
    return swab32(data);
}

int gx_pci_write_config_byte(void *dev, int where, unsigned char val)
{
    return pci_write_config_byte(dev, where, val);
}

int gx_pci_read_config_byte(void *dev, int where, unsigned char *val)
{
    return pci_read_config_byte(dev, where, val);
}

int gx_pci_write_config_word(void *dev, int where, unsigned short val)
{
    return pci_write_config_word(dev, where, val);
}

int gx_pci_read_config_word(void *dev, int where, unsigned short *val)
{
    return pci_read_config_word(dev, where, val);
}

int gx_pci_write_config_dword(void *dev, int where,
                              unsigned int val)
{
    return pci_write_config_dword(dev, where, val);
}

int gx_pci_read_config_dword(void *dev, int where,
                             unsigned int *val)
{
    return pci_read_config_dword(dev, where, val);
}

int gx_pci_alloc_irq_vectors(void *dev, unsigned int min_vecs,
                             unsigned int max_vecs, unsigned int flags)
{
    return pci_alloc_irq_vectors(dev, min_vecs, max_vecs, flags);
}

void gx_pci_free_irq_vectors(void *dev)
{
    pci_free_irq_vectors(dev);
}

int gx_pci_irq_vector(void *dev, unsigned int nr)
{
    return pci_irq_vector(dev, nr);
}

void *gx_pci_iomap(void *dev, int bar, unsigned long max)
{
    return pci_iomap(dev, bar, max);
}

void gx_pci_iounmap(void *dev, void *p)
{
    pci_iounmap(dev, p);
}

void gx_udelay(unsigned long usecs)
{
    udelay(usecs);
}

void gx_msleep(unsigned int msecs)
{
    msleep(msecs);
}

void gx_usleep_range(unsigned long min, unsigned long max)
{
    usleep_range(min, max);
}

int gx_request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags,
                   const char *name, void *dev)
{
    return request_irq(irq, handler, flags, name, dev);
}

void *gx_free_irq(unsigned int irq, void *dev_id)
{
    return free_irq(irq, dev_id);
}

unsigned char gx_readb(const volatile void *addr)
{
    return readb(addr);
}

unsigned short gx_readw(const volatile void *addr)
{
    return readw(addr);
}

unsigned int gx_readl(const volatile void *addr)
{
    return readl(addr);
}

void gx_writeb(unsigned char value, volatile void *addr)
{
    writeb(value, addr);
}

void gx_writew(unsigned short value, volatile void *addr)
{
    writew(value, addr);
}

void gx_writel(unsigned int value, volatile void *addr)
{
    writel(value, addr);
}

void *gx_filp_open(const char *filename, int flags, umode_t mode)
{
    return filp_open(filename, flags, mode);
}

int gx_filp_close(void *filp, fl_owner_t id)
{
    return filp_close(filp, id);
}

ssize_t gx_kernel_read(void *file, void *buf, size_t count, long long *pos)
{
    return kernel_read(file, buf, count, pos);
}

long long gx_i_size_read(void *inode)
{
    return i_size_read(inode);
}

void *gx_get_inode(void *file)
{
    struct file *file_temp = (struct file *)file;
    return file_temp->f_inode;
}

void *gx_get_path(void *file)
{
    struct file *file_temp = (struct file *)file;
    return &file_temp->f_path;
}

char *gx_d_path(void *path, char *buf, int buflen)
{
    return d_path(path, buf, buflen);
}

iterate_shared gx_file_get_iterate_shared(void *file)
{
    struct file *file_temp = (struct file *)file;
    return file_temp->f_op->iterate_shared;
}

long gx_ptr_err(const void *ptr)
{
    return PTR_ERR(ptr);
}

bool gx_is_err(const void *ptr)
{
    return IS_ERR(ptr);
}

long long gx_ktime_to_ns(const ktime_t kt)
{
    return ktime_to_ns(kt);
}

long long gx_ktime_to_ms(const ktime_t kt)
{
    return ktime_to_ms(kt);
}

ktime_t gx_ktime_get(void)
{
    return ktime_get();
}

ktime_t gx_ktime_us_delta(ktime_t start, ktime_t end)
{
    return ktime_us_delta(end, start);
}

void *gx_create_atomic(void)
{
    return gx_kzalloc(sizeof(atomic_t), GFP_KERNEL);
}

void gx_atomic_inc(void *v)
{
    atomic_inc((atomic_t*)v);
}

int gx_atomic_dec_return(void *v)
{
    return atomic_dec_return((atomic_t*)v);
}

unsigned int gx_hweight32(unsigned int w)
{
    return hweight32(w);
}

void gx_dev_warn(const void *dev, const char * data)
{
    dev_warn(dev, "%s", data);
}

int gx_snprintf(char *buf, size_t size, const char *fmt, ...)
{
    va_list args;
    int ret;

    va_start(args, fmt);
    ret = vsnprintf(buf, size, fmt, args); // 内核版本，安全
    va_end(args);

    return ret;
}

// v: true 继续遍历  false 停止遍历
int gx_dir_iter_enum(bool v)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
    if (v)
    {
        return 1;
    }
    else
    {
        return 0;
    }
#else
    if (v)
    {
        return 0;
    }
    else
    {
        return 1;
    }
#endif
}

int gx_get_foll_enum(int type)
{
    switch (type)
    {
    case GX_FOLL_WRITE:
        return FOLL_WRITE;
    case GX_FOLL_FORCE:
        return FOLL_FORCE;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)
    case GX_FOLL_LONGTERM:
        return FOLL_LONGTERM;
#endif
    default:
        return 0;
    }
}

bool gx_test_and_clear_bit_cond(void *data)
{
    unsigned long *flag = (unsigned long *)data;

    // 检查并清除标志位 NOTIFY_EVENT_READY
    return test_and_clear_bit(0, flag);
}

int gx_test_and_set_bit(unsigned int nr, volatile unsigned long *p)
{
    return test_and_set_bit(nr, p);
}

int gx_test_and_clear_bit(unsigned int nr, volatile unsigned long *p)
{
    return test_and_clear_bit(nr, p);
}

int gx_usb_reset_device(void *udev)
{
    return usb_reset_device(udev);
}

unsigned int gx_usb_rcvctrlpipe(void *dev, unsigned int endpoint)
{
    return usb_rcvctrlpipe(dev, endpoint);
}

unsigned int gx_usb_sndctrlpipe(void *dev, unsigned int endpoint)
{
    return usb_sndctrlpipe(dev, endpoint);
}

unsigned int gx_usb_rcvbulkpipe(void *dev, unsigned int endpoint)
{
    return usb_rcvbulkpipe(dev, endpoint);
}

unsigned int gx_usb_sndbulkpipe(void *dev, unsigned int endpoint)
{
    return usb_sndbulkpipe(dev, endpoint);
}

int gx_usb_control_msg(void *dev, unsigned int pipe, u8 request,
		    u8 requesttype, u16 value, u16 index, void *data,
		    u16 size, int timeout)
{
    return usb_control_msg(dev, pipe, request, requesttype, value, index, 
                            data, size, timeout);
}

int gx_usb_bulk_msg(void *usb_dev, unsigned int pipe,
		 void *data, int len, int *actual_length, int timeout)
{
    return usb_bulk_msg(usb_dev, pipe, data, len, actual_length, timeout);
}

int gx_usb_endpoint_is_bulk_in(void *epd)
{
    return usb_endpoint_is_bulk_in(epd);
}

int gx_usb_endpoint_type(void *epd)
{
    return usb_endpoint_type(epd);
}

int gx_usb_clear_halt(void *dev, int pipe)
{
    return usb_clear_halt(dev, pipe);
}

u16 gx_usb_maxpacket(void *udev, int pipe, int is_out)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0)
    return usb_maxpacket(udev, pipe);
#else
    return usb_maxpacket(udev, pipe, is_out);
#endif
}

void gx_usb_kill_urb(void *urb)
{
    return usb_kill_urb(urb);
}

void *gx_usb_alloc_urb(int iso_packets, unsigned int mem_flags)
{
    return usb_alloc_urb(iso_packets, mem_flags);
}

void gx_usb_free_urb(void *urb)
{
    usb_free_urb(urb);
}

void gx_usb_fill_bulk_urb(void *urb, void *dev, unsigned int pipe, void *transfer_buffer, 
						int buffer_length, usb_complete_t complete_fn, void *context)
{
    usb_fill_bulk_urb(urb, dev, pipe, transfer_buffer, buffer_length, complete_fn, context);
}

int gx_usb_submit_urb(void *urb, unsigned int mem_flags)
{
    return usb_submit_urb(urb, mem_flags);
}

void gx_set_urb_ep(void *urb, void *ep)
{
    struct urb *urb_obj = (struct urb *)urb;
    if (NULL == urb_obj)
    {
        return;
    }

    urb_obj->ep = ep;
}

void gx_set_urb_sg(void *urb, void *sg, int npages)
{
    struct urb *urb_obj = (struct urb *)urb;
    if (NULL == urb_obj)
    {
        return;
    }

    urb_obj->sg = sg;
    urb_obj->num_sgs = npages;
}

void gx_set_urb_transfer_flags(void *urb, unsigned int transfer_flags)
{
    struct urb *urb_obj = (struct urb *)urb;
    if (NULL == urb_obj)
    {
        return;
    }

    urb_obj->transfer_flags = transfer_flags;
}

void gx_set_urb_context(void *urb, void *context)
{
    struct urb *urb_obj = (struct urb *)urb;
    if (NULL == urb_obj)
    {
        return;
    }
    
    urb_obj->context = context;
}

u32 gx_get_urb_actual_length(void *urb)
{
    struct urb *urb_obj = (struct urb *)urb;
    if (NULL == urb_obj)
    {
        return 0;
    }

    return urb_obj->actual_length;
}

void* gx_get_urb_context(void *urb)
{
    struct urb *urb_obj = (struct urb *)urb;
    if (NULL == urb_obj)
    {
        return NULL;
    }
    return urb_obj->context;
}

void gx_atomic_set(void *v, int i)
{
    atomic_set(v, i);
}

int gx_atomic_read(void *v)
{
    return atomic_read(v);
}

int gx_get_sg_type_size(void)
{
    return sizeof(struct scatterlist);
}

void gx_sg_init_table(void *sgl, unsigned int nents)
{
    sg_init_table(sgl, nents);
}

void gx_sg_set_page(void *sg, void *page, unsigned int len, unsigned int offset)
{
    sg_set_page(sg, page, len, offset);
}

void _TraceFormat(int level, const char *fmt, ...)
{
    va_list args;
    char buf[TRACE_BUF_SIZE] = {0};

    va_start(args, fmt);
    vsnprintf(buf, TRACE_BUF_SIZE, fmt, args);
    va_end(args);

    switch (level)
    {
    case TRACE_LEVEL_DEBUG:
        pr_debug("%s\n", buf);
        break;

    case TRACE_LEVEL_INFO:
        pr_info("%s\n", buf);
        break;

    case TRACE_LEVEL_WARN:
        pr_warn("%s\n", buf);
        break;

    case TRACE_LEVEL_ERROR:
        pr_err("%s\n", buf);
        break;

    default:
        break;
    }
}