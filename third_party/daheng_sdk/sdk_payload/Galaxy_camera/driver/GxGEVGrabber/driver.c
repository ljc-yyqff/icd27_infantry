#include <linux/module.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include "../common/common.h"
#include "core_export.h"

#define DRIVER_NAME "GxGEVGrabber"
#define DEVICE_NAME "gx_card_dev"
#define CLASS_NAME "gx_card_class"

#define MAX_DEVICES 32
#define MAX_CHANNELS_PER_DEV (LINE_INFO_NUM_MAX * DEV_CHANNEL_NUM + 1) // 为unregister_chrdev_region
#define DEV_CHANNEL_NUM 3

static int g_major;
static struct class *pcie_class;
static DEFINE_IDA(g_card_ida);      // 自动分配index
static DEFINE_MUTEX(devnode_lock); // 防止并发创建

// 设备ID
#define CARD_VENDOR_ID_TEST 0x4448
#define CARD_DEVICE_ID_TEST 0x6869

// 设备ID表
static struct pci_device_id pci_ids[] = {
    {PCI_DEVICE(CARD_VENDOR_ID_TEST, CARD_DEVICE_ID_TEST)},
    {
        0,
    } // 终止标记
};
MODULE_DEVICE_TABLE(pci, pci_ids);

struct card_device_wrap
{
    void *card_device_private; ///< 对应card_device
    struct pci_dev *pdev;
    struct cdev cdev;
    u32 numPorts;
    bool is_request_region;
    int card_id; ///< card id 多张卡时index
};

static int pcie_open(struct inode *inode, struct file *file)
{
    struct card_device_wrap *dev;
    char *file_name = file->f_path.dentry->d_name.name;

    // 从 inode 获得 card_device 对象
    dev = container_of(inode->i_cdev, struct card_device_wrap, cdev);
    TraceDebug(DBG_INIT, "card_dev: %s opened, dev: 0x%p\n", file_name, dev);
    file->private_data = dev;
    return open_process(file_name, dev->card_device_private);
}

static int pcie_release(struct inode *inode, struct file *file)
{
    struct card_device_wrap *dev;
    char *file_name = file->f_path.dentry->d_name.name;
    // 从 inode 获得 card_device 对象
    dev = container_of(inode->i_cdev, struct card_device_wrap, cdev);
    TraceDebug(DBG_INIT, "card_dev: %s released, dev: 0x%p\n", file_name, dev);
    file->private_data = dev;
    return release_process(file_name, dev->card_device_private);
}

static long pcie_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct card_device_wrap *dev = file->private_data;
    // TraceDebug(DBG_INIT, "card_dev: device pcie_ioctl, cmd: 0x%x, dev: 0x%p\n", cmd, dev);
    char *file_name = file->f_path.dentry->d_name.name;
    return ioctl_process(file_name, dev->card_device_private, cmd, arg);
}

static unsigned int pcie_poll(struct file *file, struct poll_table_struct *wait)
{
    struct card_device_wrap *dev = file->private_data;
    char *file_name = file->f_path.dentry->d_name.name;
    return poll_process(file_name, dev->card_device_private, file, wait);
}

static const struct file_operations pcie_fops = {
    .owner = THIS_MODULE,
    .open = pcie_open,
    .release = pcie_release,
    .unlocked_ioctl = pcie_ioctl,
    .poll = pcie_poll,
};

static int create_dev_node(struct card_device_wrap *card_dev)
{
    int i, j, ret;
    dev_t devnum;
    struct device *dev_ptr;
    int total_nodes = card_dev->numPorts * DEV_CHANNEL_NUM + 1;

    mutex_lock(&devnode_lock);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
    int card_id = ida_alloc(&g_card_ida, GFP_KERNEL);
#else
    int card_id = ida_simple_get(&g_card_ida, 0, 0, GFP_KERNEL);
#endif
    mutex_unlock(&devnode_lock);

    if (card_id < 0)
        return card_id;

    card_dev->card_id = card_id;

    dev_t base_dev = MKDEV(g_major, card_id * MAX_CHANNELS_PER_DEV);

    cdev_init(&card_dev->cdev, &pcie_fops);
    card_dev->cdev.owner = THIS_MODULE;

    ret = cdev_add(&card_dev->cdev, base_dev, total_nodes);
    if (ret < 0)
    {
        ida_simple_remove(&g_card_ida, card_id);
        return ret;
    }

    // 创建每个 port/channel 的设备节点
    for (i = 0; i < card_dev->numPorts; i++)
    {
        for (j = 0; j < DEV_CHANNEL_NUM; j++)
        {
            devnum = MKDEV(g_major, card_id * MAX_CHANNELS_PER_DEV + i * DEV_CHANNEL_NUM + j);
            switch (j)
            {
            case 0:
                dev_ptr = device_create(pcie_class, NULL, devnum, NULL, "gx_gev_control%d_%d", card_id, i);
                break;
            case 1:
                dev_ptr = device_create(pcie_class, NULL, devnum, NULL, "gx_gev_event%d_%d", card_id, i);
                break;
            case 2:
                dev_ptr = device_create(pcie_class, NULL, devnum, NULL, "gx_gev_stream%d_%d", card_id, i);
                break;
            default:
                continue;
            }
            if (IS_ERR(dev_ptr))
            {
                ret = PTR_ERR(dev_ptr);
                goto fail_cleanup_devices;
            }
        }
    }

    // 创建 card 级设备节点
    devnum = MKDEV(g_major, card_id * MAX_CHANNELS_PER_DEV + card_dev->numPorts * DEV_CHANNEL_NUM);
    dev_ptr = device_create(pcie_class, NULL, devnum, NULL, "gx_gev_card%d", card_id);
    if (IS_ERR(dev_ptr))
    {
        ret = PTR_ERR(dev_ptr);
        goto fail_cleanup_devices;
    }

    return 0;

fail_cleanup_devices:
    for (i = 0; i < card_dev->numPorts; i++)
    {
        for (j = 0; j < DEV_CHANNEL_NUM; j++)
        {
            devnum = MKDEV(g_major, card_id * MAX_CHANNELS_PER_DEV + i * DEV_CHANNEL_NUM + j);
            device_destroy(pcie_class, devnum);
        }
    }
    devnum = MKDEV(g_major, card_id * MAX_CHANNELS_PER_DEV + card_dev->numPorts * DEV_CHANNEL_NUM);
    device_destroy(pcie_class, devnum);
    cdev_del(&card_dev->cdev);
    ida_simple_remove(&g_card_ida, card_id);
    return ret;
}

static void destroy_dev_node(struct card_device_wrap *card_dev)
{
    int i, j;
    int card_id = card_dev->card_id;
    dev_t devnum;
    int base_minor = card_id * MAX_CHANNELS_PER_DEV;

    for (i = 0; i < card_dev->numPorts; i++)
    {
        for (j = 0; j < DEV_CHANNEL_NUM; j++)
        {
            devnum = MKDEV(g_major, base_minor + i * DEV_CHANNEL_NUM + j);
            device_destroy(pcie_class, devnum);
        }
    }

    devnum = MKDEV(g_major, base_minor + card_dev->numPorts * DEV_CHANNEL_NUM);
    device_destroy(pcie_class, devnum);

    cdev_del(&card_dev->cdev);
    ida_simple_remove(&g_card_ida, card_id);
    TraceDebug(DBG_INIT, "destroy_dev_node success!");
}

// 设备探测（即设备插入时调用）
static int card_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct card_device_wrap *card_dev;
    int err;
    int i;
    unsigned int status = 0;

    TraceInfo(DBG_INIT, "card_dev device detected: vendor=0x%x, device=0x%x\n", id->vendor, id->device);

    // 分配私有数据
    card_dev = devm_kzalloc(&pdev->dev, sizeof(*card_dev), GFP_KERNEL);
    if (!card_dev)
    {
        return -ENOMEM;
    }

    card_dev->pdev = pdev;

    // 初始化设备对象
    status = create_card_device(&card_dev->card_device_private, pdev);
    if (0 != status)
    {
        return status;
    }

    // 启用 PCI 设备
    err = pci_enable_device(pdev);
    if (err)
    {
        TraceError(DBG_INIT, "Failed to enable PCI device\n");
        goto free;
    }

    // 当前驱动的设计是将port个数固定为8
    card_dev->numPorts = LINE_INFO_NUM_MAX;
    TraceDebug(DBG_INIT, "support ports num: %u", card_dev->numPorts);

    // 请求 BAR 资源, 避免被其他人使用
    for (i = 0; i < CARD_MAX_NUM_BARS; ++i)
    {
        err = pci_request_region(pdev, i, DRIVER_NAME);
        if (err)
        {
            TraceError(DBG_INIT, "Failed to request BAR[%d]\n", i);
            goto disable_device;
        }
    }
    card_dev->is_request_region = true;

    u64 dma_mask = dma_get_mask(&pdev->dev);
    TraceInfo(DBG_INIT, "Device DMA mask: 0x%llx", dma_mask);

    err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
    if (err)
    {
        TraceError(DBG_INIT, "No suitable DMA mask");
        goto release_region;
    }

    dma_mask = dma_get_mask(&pdev->dev);
    TraceInfo(DBG_INIT, "Device DMA mask: 0x%llx", dma_mask);

    status = init_card_device(card_dev->card_device_private);
    if (0 != status)
    {
        err = status;
        goto release_region;
    }

    pci_set_drvdata(pdev, card_dev);

    err = create_dev_node(card_dev);
    if (err)
    {
        TraceError(DBG_INIT, "create_dev_node failed: %d\n", err);
        goto release_region;
    }

    TraceInfo(DBG_INIT, "card_dev device successfully initialized\n");
    return 0;

release_region:

    if (card_dev->is_request_region)
    {
        for (i = 0; i < CARD_MAX_NUM_BARS; ++i)
            pci_release_region(pdev, i);
        card_dev->is_request_region = false;
    }

disable_device:
    if (pci_is_enabled(pdev))
        pci_disable_device(pdev);

free:
    destroy_card_device(card_dev->card_device_private);
    return err;
}

// 设备移除（即设备拔出时调用）
static void card_remove(struct pci_dev *pdev)
{
    int i;
    struct card_device_wrap *card_dev = pci_get_drvdata(pdev);
    TraceInfo(DBG_INIT, "Removing card_dev device\n");
    destroy_dev_node(card_dev);
    uninit_card_device(card_dev->card_device_private);

    if (card_dev->is_request_region)
    {
        for (i = 0; i < CARD_MAX_NUM_BARS; ++i)
            pci_release_region(pdev, i);
        card_dev->is_request_region = false;
    }

    if (pci_is_enabled(pdev))
        pci_disable_device(pdev);

    destroy_card_device(card_dev->card_device_private);
    devm_kfree(&pdev->dev, card_dev);
    TraceInfo(DBG_INIT, "Remove card_dev success\n");
}

int card_suspend(struct device *dev)
{
    TraceInfo(DBG, DRIVER_NAME" suspend.");
    return 0;
}

int card_resume(struct device *dev)
{
    TraceInfo(DBG, DRIVER_NAME" resume.");
    struct pci_dev *pdev = to_pci_dev(dev);
    struct card_device_wrap *card_dev = pci_get_drvdata(pdev);
    return resume_card(card_dev->card_device_private);
}

static const struct dev_pm_ops card_pm_ops = {
    .suspend = card_suspend,
    .resume  = card_resume,
};

// PCI 驱动结构体
static struct pci_driver card_driver = {
    .name = DRIVER_NAME,
    .id_table = pci_ids,
    .probe = card_probe,
    .remove = card_remove,
    .driver = {
        .pm = &card_pm_ops,
    }
};

// 模块加载和卸载
static int __init card_init(void)
{
    dev_t dev;
    int ret;

    ret = alloc_chrdev_region(&dev, 0, MAX_DEVICES * MAX_CHANNELS_PER_DEV, DRIVER_NAME);
    if (ret < 0)
        return ret;

    g_major = MAJOR(dev);

#if LINUX_VERSION_CODE <= KERNEL_VERSION(6, 3, 0)
    pcie_class = class_create(THIS_MODULE, DRIVER_NAME);
#else
    pcie_class = class_create(DRIVER_NAME);
#endif
    if (IS_ERR(pcie_class))
    {
        unregister_chrdev_region(MKDEV(g_major, 0), MAX_DEVICES * MAX_CHANNELS_PER_DEV);
        return PTR_ERR(pcie_class);
    }

    ida_init(&g_card_ida);

    return pci_register_driver(&card_driver);
}

static void __exit card_exit(void)
{
    pci_unregister_driver(&card_driver);
    class_destroy(pcie_class);
    unregister_chrdev_region(MKDEV(g_major, 0), MAX_DEVICES * MAX_CHANNELS_PER_DEV);
    ida_destroy(&g_card_ida);
}

module_init(card_init);
module_exit(card_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daheng");
MODULE_DESCRIPTION("GxGEVGrabber");
MODULE_VERSION("1.0");