#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/lcm.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/poll.h>
#include <linux/pci.h>
#ifdef VERSION_COMPATIBILITY
    #include <linux/version.h>
#else
    #include <generated/uapi/linux/version.h>
#endif
#include "core_export.h"
#include "../common/common.h"
#include "u3v_def.h"

#define GALAXY_U3D_VID	0x2BA2
#define GALAXY_U3D_PID	0x4455 

#define DRIVER_NAME "GxU3VBase"
#define DEVICE_NAME "gx_u3v_dev"

struct u3v_device {
    struct usb_device *udev;
    struct usb_interface *intf;
    struct device *device;
    struct kref kref;
    struct u3v_device_info *u3v_info; /* device attributes */
    struct usb_driver *u3v_driver;
    // int device_state;
    // bool stalling_disabled;
    // atomic_t device_available;
    bool device_connected;	
    struct u3v_interface_info control_info;
    struct u3v_interface_info event_info;
    struct u3v_interface_info stream_info;

    unsigned short vid;
    unsigned short did;
};

void *gx_get_udev(void *u3v_dev)
{
    struct u3v_device* dev = (struct u3v_device*)u3v_dev;
    if (NULL == dev)
    {
        return NULL;
    }

    return dev->udev;
}

unsigned short gx_get_parent_vid(void *u3v_dev)
{
    struct u3v_device* dev = (struct u3v_device*)u3v_dev;
    if (NULL == dev)
    {
        return 0;
    }

    return dev->vid;
}

unsigned short gx_get_parent_did(void *u3v_dev)
{
    struct u3v_device* dev = (struct u3v_device*)u3v_dev;
    if (NULL == dev)
    {
        return 0;
    }

    return dev->did;
}

struct u3v_interface_info *gx_get_control_interface_info(void *u3v_dev)
{
    struct u3v_device* dev = (struct u3v_device*)u3v_dev;
    if (NULL == dev)
    {
        return NULL;
    }

    return &dev->control_info;
}

struct u3v_interface_info *gx_get_event_interface_info(void *u3v_dev)
{
    struct u3v_device* dev = (struct u3v_device*)u3v_dev;
    if (NULL == dev)
    {
        return NULL;
    }
    
    return &dev->event_info;
}

struct u3v_interface_info *gx_get_stream_interface_info(void *u3v_dev)
{
    struct u3v_device* dev = (struct u3v_device*)u3v_dev;
    if (NULL == dev)
    {
        return NULL;
    }

    return &dev->stream_info;
}

/* probe/disconnect and file operations */
static int u3v_probe(struct usb_interface *interface,
             const struct usb_device_id *id);
static void u3v_disconnect(struct usb_interface *interface);
static int u3v_open(struct inode *inode, struct file *filp);
static int u3v_release(struct inode *inode, struct file *filp);
static long u3v_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static unsigned int u3v_poll (struct file *file, struct poll_table_struct *wait);

/* helper functions for probe */
static int enumerate_u3v_interfaces(struct u3v_device *u3v);
static int populate_u3v_properties(struct u3v_device *u3v);
static struct usb_endpoint_descriptor *find_bulk_endpoint(
    struct usb_host_interface *iface_desc, enum direction_t dir);

/* helper function for reset_pipe */
// static int reset_endpoint(struct u3v_device *u3v,
// 	struct usb_endpoint_descriptor *ep);

static const struct file_operations fops = {
    .owner = THIS_MODULE,
/*	.read not implemented  */
/*	.write not implemented */
    .open = u3v_open,
    .release = u3v_release,
    .unlocked_ioctl = u3v_ioctl,
    .poll = u3v_poll,
    .llseek = default_llseek,
};

static struct usb_class_driver u3v_class = {
    .name =		DEVICE_NAME"%d",
    .fops =		&fops,
    .minor_base =	U3V_MINOR_BASE,
};

static struct usb_device_id u3v_table[] = {
    { USB_DEVICE(GALAXY_U3D_VID, GALAXY_U3D_PID)},
    {} /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, u3v_table);

static struct usb_driver u3v_driver = {
    .name = DRIVER_NAME,
    .id_table = u3v_table,
    .probe = u3v_probe,
    .disconnect = u3v_disconnect,
};

static void u3v_delete(struct kref *kref)
{
    struct u3v_device *u3v = to_u3v_device(kref);
    if (u3v == NULL)
    {
        return;
    }

    usb_put_dev(u3v->udev);
    kfree(u3v->u3v_info);
    kfree(u3v);
}

/*
 * u3v_open - called when a process tries to open the device file
 */
static int u3v_open(struct inode *inode, struct file *filp)
{
    struct usb_interface *intf;
    struct u3v_device *u3v;
    void *u3v_obj;

    intf = usb_find_interface(&u3v_driver, iminor(inode));
    if (!intf) 
    {
        TraceError(u3v, "%s: cannot find device for minor %d\n",
            __func__, iminor(inode));
        return -ENODEV;
    }

    u3v = usb_get_intfdata(intf);
    if (NULL == u3v)
    {
        return -ENODEV;
    }

    kref_get(&u3v->kref);

    u3v_obj = open_process(u3v);
    if (NULL == u3v_obj)
    {
        kref_put(&u3v->kref, u3v_delete);
        return -ENOMEM;
    }

    filp->private_data = u3v_obj;

    return 0;
}

/*
 * u3v_release - called when a process closes the device file
 */
static int u3v_release(struct inode *inode, struct file *filp)
{
    struct u3v_device *u3v;
    if (!filp || !filp->private_data)
    {
        return -EINVAL;	
    }

    u3v = get_u3v(filp->private_data);
    release_process(filp->private_data);

    if (NULL != u3v)
    {
        kref_put(&u3v->kref, u3v_delete);
    }
    filp->private_data = NULL;
    return 0;
}

static long u3v_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    if (!filp || !filp->private_data)
    {
        return -EINVAL;
    }

    return ioctl_process(filp->private_data, cmd, arg);
}

/*
 * find_bulk_endpoint - helper function to iterate through endpoints and
 *	return a bulk endpoint that has the correct direction as specified
 *	by dir
 * @iface_desc: interface descriptor
 * @dir_in: true if looking for an in endpoint, false for an out endpoint
 */
struct usb_endpoint_descriptor *find_bulk_endpoint(struct usb_host_interface
    *iface_desc, enum direction_t dir)
{
    struct usb_endpoint_descriptor *ep;
    int i;

    if (NULL == iface_desc)
    {
        return NULL;
    }

    for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) 
    {
        ep = &iface_desc->endpoint[i].desc;
        if (((dir == dir_in) && usb_endpoint_is_bulk_in(ep)) ||
            ((dir == dir_out) && usb_endpoint_is_bulk_out(ep))) 
        {
            return ep;
        }
    }

    return NULL;
}


/*
 * populate_u3v_properties - this helper function copies necessary
 *	USB3 data to our u3v_device_info struct
 * @u3v: pointer to the u3v struct
 */
static int populate_u3v_properties(struct u3v_device *u3v)
{
    struct usb_host_interface *alts;
    unsigned char *buffer;
    int buflen;
    struct usb_device *udev;
    struct u3v_device_info *u3v_info;

    if (NULL == u3v || NULL == u3v->udev || NULL == u3v->u3v_info ||
        NULL == u3v->intf || NULL == u3v->intf->cur_altsetting ||
        NULL == u3v->intf->cur_altsetting->extra)
    {
        return -EINVAL;
    }

    udev = u3v->udev;
    u3v_info = u3v->u3v_info;

    alts = u3v->intf->cur_altsetting;
    buffer = alts->extra;
    buflen = alts->extralen;

    if ((buflen < MIN_U3V_INFO_LENGTH) ||
        (buffer[1] != U3V_INTERFACE) ||
        (buffer[2] != U3V_DEVICEINFO)) 
    {
        TraceError(u3v, "%s: Failed to get a proper Camera Info descriptor.\n", __func__);
        TraceError(u3v, "Descriptor Type = 0x%02X (Expected 0x%02X)\n", buffer[1], U3V_INTERFACE);
        TraceError(u3v, "Descriptor SubType = 0x%02X (Expected 0x%02X)\n", buffer[2], U3V_DEVICEINFO);
        TraceError(u3v, "Descriptor Length = 0x%02X (Expected 0x%02X)\n", buflen, MIN_U3V_INFO_LENGTH);
        return -EINVAL;
    }

    /* uint32_t gen_cp_version */
    u3v_info->gen_cp_version = le32_to_cpup((const __le32 *)&buffer[3]);

    /* uint32_t u3v_version */
    u3v_info->u3v_version = le32_to_cpup((const __le32 *)&buffer[7]);

    /* char device_guid [U3V_MAX_STR] */
    if (buffer[11] != 0)
    {
        usb_string(udev, buffer[11], u3v_info->device_guid,
            sizeof(u3v_info->device_guid));
    }

    /* char vendor_name [U3V_MAX_STR] */
    if (buffer[12] != 0)
    {
        usb_string(udev, buffer[12], u3v_info->vendor_name,
            sizeof(u3v_info->vendor_name));
    }

    /* char model_name [U3V_MAX_STR] */
    if (buffer[13] != 0)
    {
        usb_string(udev, buffer[13], u3v_info->model_name,
            sizeof(u3v_info->model_name));
    }

    /* char family_name [U3V_MAX_STR] */
    if (buffer[14] != 0)
    {
        usb_string(udev, buffer[14], u3v_info->family_name,
            sizeof(u3v_info->family_name));
    }

    /* char device_version [U3V_MAX_STR] */
    if (buffer[15] != 0)
    {
        usb_string(udev, buffer[15], u3v_info->device_version,
            sizeof(u3v_info->device_version));
    }

    /* char manufacturer_info [U3V_MAX_STR] */
    if (buffer[16] != 0)
    {
        usb_string(udev, buffer[16], u3v_info->manufacturer_info,
            sizeof(u3v_info->manufacturer_info));
    }

    /* char serial_number_u3v [U3V_MAX_STR] */
    if (buffer[17] != 0)
    {
        usb_string(udev, buffer[17], u3v_info->serial_number_u3v,
            sizeof(u3v_info->serial_number_u3v));
    }

    /* char user_defined_name [U3V_MAX_STR] */
    if (buffer[18] != 0)
    {
        usb_string(udev, buffer[18], u3v_info->user_defined_name,
            sizeof(u3v_info->user_defined_name));
    }

    return 0;
}

/*
 * enumerate_u3v_interfaces - called by probe to claim the interfaces and
 *	store their endpoint information
 * @u3v - pointer to the u3v device struct
 */
static int enumerate_u3v_interfaces(struct u3v_device *u3v)
{
    struct usb_device *udev;
    struct usb_interface *tmp;
    struct usb_host_interface *iface_desc;
    int num_interfaces, i;
    enum direction_t in = dir_in;
    enum direction_t out = dir_out;
    int ret = 0;

    if (NULL == u3v || NULL == u3v->udev || NULL == u3v->udev->actconfig)
    {
        return -EINVAL;
    }

    udev = u3v->udev;
    num_interfaces = udev->actconfig->desc.bNumInterfaces;

    //只有Event和Stream interface需要进行claim, 在probe前Control interface已被claim,
    //否则无法进入probe
    for (i = 0; i < num_interfaces; i++) 
    {
        tmp = usb_ifnum_to_if(udev, i);
        if (NULL == tmp || NULL == tmp->cur_altsetting)
        {
            return -EINVAL;
        }

        iface_desc = tmp->cur_altsetting;
        switch (iface_desc->desc.bInterfaceProtocol) 
        {
        case U3V_INTERFACE_PROTOCOL_CONTROL:
        {
            u3v->control_info.idx = i;
            u3v->control_info.bulk_in = find_bulk_endpoint(iface_desc, in);
            u3v->control_info.bulk_out = find_bulk_endpoint(iface_desc, out);
            if (NULL == u3v->control_info.bulk_in || NULL == u3v->control_info.bulk_out)
            {
                return -EINVAL;
            }

            TraceDebug(u3v, "control bulkin: 0x%x, bulkout: 0x%x", 
            u3v->control_info.bulk_in->bEndpointAddress, u3v->control_info.bulk_out->bEndpointAddress);
        }
            break;
        case U3V_INTERFACE_PROTOCOL_EVENT:
        {
            ret = usb_driver_claim_interface(&u3v_driver, tmp, u3v);
            if (ret != 0) 
            {
                TraceError(u3v, "%s: event interface for device %d is already claimed\n", __func__,
                    u3v->udev->devnum);
                return -EBUSY;
            }

            u3v->event_info.idx = i;
            u3v->event_info.bulk_in = find_bulk_endpoint(iface_desc, in);
            u3v->event_info.bulk_out = NULL;
            if (NULL == u3v->event_info.bulk_in)
            {
                return -EINVAL;
            }
            TraceDebug(u3v, "event bulkin: 0x%x", u3v->event_info.bulk_in->bEndpointAddress);
        }
            break;
        case U3V_INTERFACE_PROTOCOL_STREAM:
        {
            /* claim stream interface */
            ret = usb_driver_claim_interface(&u3v_driver, tmp, u3v);
            if (ret != 0)
            {
                TraceError(u3v, "%s: stream interface for device %d is already claimed\n", __func__,
                    u3v->udev->devnum);
                return -EBUSY;
            }

            u3v->stream_info.idx = i;
            u3v->stream_info.bulk_in = find_bulk_endpoint(iface_desc, in);
            u3v->stream_info.bulk_out = NULL;
            if (NULL == u3v->stream_info.bulk_in)
            {
                return -EINVAL;
            }
            TraceDebug(u3v, "stream bulkin: 0x%x", u3v->stream_info.bulk_in->bEndpointAddress);
        }
            break;
        }
    }
    return 0;
}

/*
 * u3v_probe - called by the usb core if a device has an interface that
 *	matches the device table
 * @interface: matching usb_interface
 * @id: matching usb_device_id
 */
static int u3v_probe(struct usb_interface *interface,
    const struct usb_device_id *id)
{
    struct u3v_device *u3v;
    struct u3v_device_info *u3v_info;
    struct device *controller;
    struct pci_dev *pdev;
    int ret = 0;
    
    if (NULL == interface || NULL == id || NULL == interface->cur_altsetting)
    {
        return -EINVAL;
    }

    TraceInfo(u3v, "probe vendor: 0x%x, product: 0x%x, ifnum: %d\n", id->idVendor, id->idProduct,
            interface->cur_altsetting->desc.bInterfaceNumber);


    if (U3V_INTERFACE_PROTOCOL_CONTROL != interface->cur_altsetting->desc.bInterfaceNumber)
    {
        return -ENODEV;
    }

    /* Allocate memory for the device. */
    u3v = kzalloc(sizeof(*u3v), GFP_KERNEL);
    if (u3v == NULL)
    {
        return -ENOMEM;
    }

    /* Allocate memory for the U3V device info. */
    u3v_info = kzalloc(sizeof(*u3v_info), GFP_KERNEL);
    if (u3v_info == NULL) 
    {
        kfree(u3v);
        return -ENOMEM;
    }

    /* Initialize members of struct u3v_device */
    u3v->udev = usb_get_dev(interface_to_usbdev(interface));
    u3v->intf = usb_get_intf(interface);
    u3v->device = &u3v->udev->dev;
    u3v->u3v_driver = &u3v_driver;
    u3v->u3v_info = u3v_info;
    kref_init(&u3v->kref);	
    u3v->device_connected = true;

    /* get vid and did */
    controller = u3v->udev->bus->controller;
    if (controller->bus == &pci_bus_type) 
    {
        pdev = to_pci_dev(controller);
        u3v->vid = pdev->vendor;
        u3v->did = pdev->device;
    }

    /* Populate u3v_info */
    ret = populate_u3v_properties(u3v);
    if (ret < 0) 
    {
        TraceError(u3v, "populate_u3v_properties failed, ret: %d\n", ret);
        goto error;
    }

    ret = enumerate_u3v_interfaces(u3v);
    if (ret < 0)
    {
        goto error;
    }

    ret = usb_register_dev(u3v->intf, &u3v_class);
    if (ret < 0) 
    {
        TraceError(u3v, "%s: Failed to register device\n", __func__);
        goto error;
    }

    TraceInfo(u3v, "%s: Registering device %d\n", DRIVER_DESC, u3v->udev->devnum);

    /* Save our data pointer in this interface device */
    usb_set_intfdata(interface, u3v);

    return ret;
    
error:
    kfree(u3v->u3v_info);
    kfree(u3v);
    return ret;
}

/*
 * u3v_disconnect - called by the usb core when the interface has been
 *	removed from the system or the driver is being unloaded
 * @interface: interface to be removed
 */
static void u3v_disconnect(struct usb_interface *interface)
{
    struct u3v_device *u3v;
    if (NULL == interface)
    {
        return;
    }

    u3v = usb_get_intfdata(interface);
    if (NULL == u3v)
    {
        return;
    }

    if (interface->cur_altsetting->desc.bInterfaceNumber == U3V_INTERFACE_PROTOCOL_CONTROL) 
    {
        TraceInfo(u3v, "%s: U3V device removed\n", __func__);
        usb_deregister_dev(interface, &u3v_class);
        u3v->device_connected = false;
        kref_put(&u3v->kref, u3v_delete);
    }
}

static unsigned int u3v_poll (struct file *file, struct poll_table_struct *wait)
{
    return poll_process(file->private_data, file, wait);
}

/*
 * u3v_init - register the u3v driver
 */
static int __init u3v_init(void)
{
    int result;

    /* register this driver with the USB subsystem */
    result = usb_register(&u3v_driver);
    if (result == 0)
    {
        TraceInfo(u3v, DRIVER_DESC"\n");
    }

    return result;
}

/*
 * u3v_exit - deregister the u3v driver
 */
static void __exit u3v_exit(void)
{
    usb_deregister(&u3v_driver);
}

module_init(u3v_init);
module_exit(u3v_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daheng");
MODULE_DESCRIPTION("GxU3VBase");
MODULE_VERSION("1.0");
