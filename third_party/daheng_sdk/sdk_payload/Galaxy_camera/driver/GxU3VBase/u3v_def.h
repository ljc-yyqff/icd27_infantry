#ifndef GX_U3V_DEF_H
#define GX_U3V_DEF_H

#include "../common/common.h"

/* General device info */
#define DRIVER_DESC "Galaxy USB3 Vision Driver"
#define U3V_DEVICE_CLASS 0xEF
#define U3V_DEVICE_SUBCLASS 0x02
#define U3V_DEVICE_PROTOCOL 0x01
#define U3V_INTERFACE_CLASS 0xEF
#define U3V_INTERFACE_SUBCLASS 0x05
#define U3V_INTERFACE_PROTOCOL_CONTROL 0x00
#define U3V_INTERFACE_PROTOCOL_EVENT 0x01
#define U3V_INTERFACE_PROTOCOL_STREAM 0x02
#define U3V_INTERFACE 0x24
#define U3V_DEVICEINFO 0x01
#define MIN_U3V_INFO_LENGTH 20
#define U3V_MINOR_BASE 200
#define U3V_MAX_STR 64
#define	U3V_DEV_DISCONNECTED 1

enum direction_t {
	dir_in,
	dir_out
};

struct u3v_interface_info 
{
	u8 idx;
	struct usb_endpoint_descriptor *bulk_in;
	struct usb_endpoint_descriptor *bulk_out;
};


#define to_u3v_device(d) container_of(d, struct u3v_device, kref)

/*
 * Each u3v_device contains a u3v_device_info struct to store
 * device attributes
 */
struct u3v_device_info 
{
	u32 gen_cp_version;
	u32 u3v_version;
	char device_guid[U3V_MAX_STR];
	char vendor_name[U3V_MAX_STR];
	char model_name[U3V_MAX_STR];
	char family_name[U3V_MAX_STR];
	char device_version[U3V_MAX_STR];
	char manufacturer_info[U3V_MAX_STR];
	char serial_number_u3v[U3V_MAX_STR];
	char user_defined_name[U3V_MAX_STR];
	// u8 speed_support;
};


#endif
