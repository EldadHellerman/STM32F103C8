#ifndef USB_MASS_STORAGE
#define USB_MASS_STORAGE

#include "usb.h"

#define USB_CLASS_MASS_STORAGE              0x08
#define MASS_STORAGE_SUBCLASS_SCSI 0x06
#define MASS_STORAGE_PROTOCOL_BBB 0x50

struct usb_descriptor_configuration descriptor_configuration = {9,USB_DESCRIPTOR_CONFIGURATION,34,2,1,0,0x80,50};
struct usb_descriptor_interface descriptor_interface_2 = {9,USB_DESCRIPTOR_INTERFACE,1,0,2,USB_INTERFACE_CLASS_MASS_STORAGE,MASS_STORAGE_SUBCLASS_SCSI,MASS_STORAGE_PROTOCOL_BBB,0};
struct usb_descriptor_endpoint descriptor_endpoint = {7,USB_DESCRIPTOR_ENDPOINT,0x81,3,16,100}; //BULK OUT endpoint
struct usb_descriptor_endpoint descriptor_endpoint = {7,USB_DESCRIPTOR_ENDPOINT,0x81,3,16,100}; //BULK IN endpoint

#endif
