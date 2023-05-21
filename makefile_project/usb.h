#ifndef USB_H
#define USB_H

#include "stm32f103c8.h"

//windows USB enumeration process: https://techcommunity.microsoft.com/t5/microsoft-usb-blog/how-does-usb-stack-enumerate-a-device/ba-p/270685

//do all setup and configuration in usb.c and provide:
//  a way to register callback functions for deferent events
//  a way to set endpoints, and give it address after last endpoint
//  a way to init usb with selected features
//  a way to register descriptors for device, configuration, string, interface, endpoint, etc...
//  a way and read data from buffer
//  endpoint struct, with its own status for data sending - send data function is given endpoint struct pointer.

#define USB_EP_OUT             0x00
#define USB_EP_IN              0x01
#define USB_EP_BULK            0x00
#define USB_EP_CONTROL         0x01
#define USB_EP_ISOCHRONOUS     0x02
#define USB_EP_INTERRUPT       0x03
#define USB_EP_DISABLED        0x00
#define USB_EP_STALL           0x01
#define USB_EP_NAK             0x02
#define USB_EP_VALID           0x03

#define USB_EVENT_RESET        (0x01 << 0)
#define USB_EVENT_SUSPEND      (0x01 << 1)
#define USB_EVENT_WAKEUP       (0x01 << 2)

typedef struct{ //usb_ep
  uint8_t RESERVED : 1; //set to one to identify this is a valid ep data and not an undefined 0x00
  uint8_t number : 7;
  uint8_t direction : 1;
  uint8_t address : 7;
} usb_ep;
//TODO maybe make ep an ADT

typedef void (*usb_event_cb)(uint8_t event);
typedef void (*usb_cb_rx)(char* data, uint16_t size);
typedef void (*usb_cb_tx)(void);

void usb_init(void);
void usb_print_status(void);
void usb_add_event_cb(usb_event_cb cb);
void usb_add_descriptor(char *desc);
void usb_add_special_descriptor(uint16_t length, uint8_t type, char *desc); //not included when sending configuration description
usb_ep usb_add_endpoint(uint8_t direction, uint8_t address, uint8_t type, uint16_t maxPacketSize);
void usb_endpoint_set_status(usb_ep ep, int status);

void usb_add_endpoint_cb_tx(usb_ep ep, usb_cb_tx cb); //tx_cb is called for every data packet sent
void usb_add_endpoint_cb_rx(usb_ep ep, usb_cb_rx cb); //rx_cb is called for every data packet recived
void usb_send(usb_ep ep, uint8_t *data, uint16_t size);
/* TODO add support for buffered rx and tx
void usb_add_endpoint_cb_buffered_rx(usb_ep ep, usb_tx_cb cb, char* buffer, uint16_t buffer_size); //data is buffered and transaction_cb is called only when transfer completes.
void usb_send_buffered(usb_ep ep, uint8_t *data, uint16_t size);
struct endpoint_transaction_state{ //endpoint state should be both for data rx and tx, since both can be split across multiple transactions
  uint8_t *buffer;   //Tx/Rx buffer start
  uint8_t *buffer_current;   //Tx/Rx buffer next location to send/reciev
  uint16_t waiting_for_zlp : 1; //Tx - ZLP as ack
  uint16_t need_to_send_zlp : 1; //Tx - data is a multiple of wMaxPacketSize so ZLP is needed, Rx - need to send ack
  uint16_t bytes_left : 14; //Tx - amount of bytes left, Rx - available buffer size left
}
*/
//TODO add functions like add_interface(interface parameters), add_endpoint(endpoint parameters), and generate proper descriptor for them.
//      then remove add_descriptor function, except for special descriptor like hid report desciptor
//TODO maybe do:
//  typedef char *usb_descriptor;
//  typdef usb_descriptor struct usb_descriptor_device{uint8_t x; uint8_t y};
//and then all of the descriptors are passable as a usb_descriptor type.

//USB Specifications Structures and data:
struct usb_setup_request_pma{
  uint32_t bmRequestType : 8;
  uint32_t bRequest : 8;
  uint32_t RESERVED0 : 16;
  union{
    struct{
      uint32_t wValue : 16;
      uint32_t RESERVED1 : 16;
    };
    struct{
      uint32_t wValue_l : 8;
      uint32_t wValue_h : 8;
      uint32_t RESERVED2 : 16; //TODO try to change its name to RESERVED1 as well, it may work since they share the same memory
    };
  };

  uint32_t wIndex : 16;
  uint32_t RESERVED3 : 16;
  uint32_t wLength : 16;
  uint32_t RESERVED4 : 16;
};

struct usb_setup_request{
  uint8_t bmRequestType;
  uint8_t bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
};

struct usb_descriptor_device{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t bcdUSB;
  uint8_t bDeviceClass;
  uint8_t bDeviceSubClass;
  uint8_t bDeviceProtocol;
  uint8_t bMaxPacketSize0;
  uint16_t idVendor;
  uint16_t idProduct;
  uint16_t bcdDevice;
  uint8_t iManufacturer;
  uint8_t iProduct;
  uint8_t iSerialNumber;
  uint8_t bNumConfigurations;
};

struct usb_descriptor_configuration{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t wTotalLength;
  uint8_t bNumInterfaces;
  uint8_t bConfigurationValue;
  uint8_t iConfiguration;
  uint8_t bmAttributes;
  uint8_t bMaxPower;
};

struct usb_descriptor_interface{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bInterfaceNumber;
  uint8_t bAlternateSetting;
  uint8_t bNumEndpoints;
  uint8_t bInterfaceClass;
  uint8_t bInterfaceSubClass;
  uint8_t bInterfaceProtocol;
  uint8_t iInterface;
};

struct usb_descriptor_endpoint{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bEndpointAddress;
  uint8_t bmAttributes;
  uint16_t wMaxPacketSize;
  uint8_t bInterval;
};

struct usb_supported_languages{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t *wLANGID;
};

struct usb_string{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t *string_unicode;
};

#define USB_SETUP_REQUESTTYPE_RECIPIENT               0x1F <<  0
#define USB_SETUP_REQUESTTYPE_TYPE                    0x03 <<  5
#define USB_SETUP_REQUESTTYPE_DIRECTION               0x01 <<  7
#define USB_SETUP_REQUEST_GET_STATUS                  0
#define USB_SETUP_REQUEST_CLEAR_FEATURE               1
#define USB_SETUP_REQUEST_SET_FEATURE                 3
#define USB_SETUP_REQUEST_SET_ADDRESS                 5
#define USB_SETUP_REQUEST_GET_DESCRIPTOR              6
#define USB_SETUP_REQUEST_SET_DESCRIPTOR              7
#define USB_SETUP_REQUEST_GET_CONFIGURATION           8
#define USB_SETUP_REQUEST_SET_CONFIGURATION           9
#define USB_SETUP_REQUEST_GET_INTERFACE               10
#define USB_SETUP_REQUEST_SET_INTERFACE               11
#define USB_SETUP_REQUEST_SYNCH_FRAME                 12

#define USB_DESCRIPTOR_DEVICE                         1
#define USB_DESCRIPTOR_CONFIGURATION                  2
#define USB_DESCRIPTOR_STRING                         3
#define USB_DESCRIPTOR_INTERFACE                      4
#define USB_DESCRIPTOR_ENDPOINT                       5
#define USB_DESCRIPTOR_DEVICE_QUALIFIER               6
#define USB_DESCRIPTOR_OTHER_SPEED_CONFIGURATOIN      7
#define USB_DESCRIPTOR_INTERFACE_POWER                8

#define USB_1_0                                       0x0100
#define USB_1_1                                       0x0110
#define USB_2_0                                       0x0200

#endif
