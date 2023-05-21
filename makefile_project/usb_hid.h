#ifndef USB_HID_H
#define USB_HID_H

#include "usb.h"
#include "printf.h"
/* whats HID does on top of USB?
*class defined constants
*descriptors for:
*   interface
*   HID
*   endpoint
*   HID report desc
*data Rx/Tx:
*   adds a cb/ listener for its endpoints + special/class data on endpoint 0
*   send data back in response to data recieved
*set it endpoints to nak/stall/ack
*/

#define USB_CLASS_HID                       0x03
#define USB_DESCRIPTOR_HID                  0x21
#define USB_DESCRIPTOR_HID_REPORT           0x22
#define HID_PROTOCOL_KEYBOARD               1
#define HID_PROTOCOL_MOUSE                  2

struct usb_descriptor_hid{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t bcdHID;
  uint8_t bCountryCode;
  uint8_t bNumDescriptors;
  uint8_t bDescriptorType_2;
  uint16_t wDescriptorLength_2;
};

void hid_init(void);

/*
advanced:
    void hid_add_mouse
    void hid_add_keyboard
    void hid_add_volume
simple:
    hid_mouse_move(x,y)
    hid_keyboard_type(char *s)
    hid_mouse_press
    hid_mouse_release
    hid_mouse_click
    hid_keyboard_press
    hid_keyboard_release
    hid_keyboard_click
*/
#endif
