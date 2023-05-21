#include "usb_hid.h"

#define HID_REPORT_DESC_LEN     118

static usb_ep ep;
static struct usb_descriptor_interface descriptor_interface = {9,USB_DESCRIPTOR_INTERFACE,0,0,1,USB_CLASS_HID,0,0,4};
static struct usb_descriptor_hid descriptor_hid = {9,USB_DESCRIPTOR_HID,0x0101,0,1,USB_DESCRIPTOR_HID_REPORT,HID_REPORT_DESC_LEN};
static struct usb_descriptor_endpoint descriptor_endpoint = {7,USB_DESCRIPTOR_ENDPOINT,0x81,3,16,100}; //Interrupt IN endpoint
static uint8_t HID_report_descriptor_length = HID_REPORT_DESC_LEN;
static uint8_t HID_report_descriptor[HID_REPORT_DESC_LEN] = {   0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x85, 0x01,
                                                           0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00,
                                                           0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
                                                           0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x95, 0x03,
                                                           0x75, 0x08, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00,
                                                           0xC0,
                                                           0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0xA1, 0x00,
                                                           0x85, 0x02, 0x05, 0x09, 0x19, 0x01, 0x29, 0x03,
                                                           0x15, 0x00, 0x25, 0x01, 0x95, 0x03, 0x75, 0x01,
                                                           0x81, 0x02, 0x95, 0x01, 0x75, 0x05, 0x81, 0x01,
                                                           0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x15, 0x81,
                                                           0x25, 0x7F, 0x95, 0x02, 0x75, 0x08, 0x81, 0x06,
                                                           0xC0, 0xC0,
                                                           0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x03,
                                                           0x09, 0xe2, 0x09, 0xe9, 0x09, 0xea, 0x95, 0x03,
                                                           0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05,
                                                           0x81, 0x01, 0xC0};

static char report_current = 0; //used for tracking - 0 to send mouse report, 1 to send keyboard report
static int report_keyboard_length = 5;
static uint8_t report_keyboard[5] = {1,0,0,0,0}; //report id, modifiers bitmap, data
static int report_mouse_length = 4;
static uint8_t report_mouse[4] = {2,0,0,0}; //report id, data
static int report_volume_length = 2;
static uint8_t report_volume[2] = {3,0x01}; //report id, modifiers bitmap, data
static char *text = "this is a mixed keyboard and mouse ShidS device.\n";
static char *p = 0;

static void event_cb(uint8_t e){
  printf("HID e_cb - ");
  switch(e){
    case USB_EVENT_RESET:
      printf("reset\n");
      break;
    case USB_EVENT_SUSPEND:
      printf("suspend\n");
      break;
    case USB_EVENT_WAKEUP:
      printf("wakeup\n");
      break;
  }
}

static void hid_setIdle(void){
  usb_send(ep ,report_mouse, 4);
}

/*static void setIdle(){
  usb_send(report_mouse, 4);
}*/

static void cb_tx(){
  if(report_current == 0){
    usb_send(ep, report_mouse, report_mouse_length);
  }else if(report_current == 1){
    usb_send(ep, report_keyboard, report_keyboard_length);
  }else if(report_current == 2){
    usb_send(ep, report_volume, report_volume_length);
  }
  report_current++;
  if(report_current == 3) report_current = 0;
  /*
  // if(DEBUG) printf("d!\n");
  if(report_current == 0){
    if(mouse_d) mouse_t++; else mouse_t--;
    if((mouse_t == 20) || (mouse_t == 0)){
      if(mouse_d == 0){
        mouse_d = 1;
        report_mouse[3] = -10;
      }else{
        mouse_d = 0;
        report_mouse[3] = 10;
      }
    }
    for(int i=0; i<report_mouse_length; i++) buffer_set(0x80+i, report_mouse[i]);
    USB_BTABLE->ep[1].tx_count = report_mouse_length;
    report_current = 1;
  }else if(report_current == 1){
    if(p == 0) p = text;
    if(report_keyboard[2] == 0){
      if((*p <= 'z') && (*p >= 'a')) report_keyboard[2] = (*p++) - 'a' + 0x04;
      else{
        if(*p == ' ') report_keyboard[2] = 0x2C;
        if(*p == '\n') report_keyboard[2] = 0x28;
        if(*p == 'C') report_keyboard[2] = 0x39;
        if(*p == 'S') report_keyboard[1] = (report_keyboard[1] == 0) ? 0x02 : 0;
        p++;
      }
      if(*p == 0) p = text;
    }else{
      report_keyboard[2] = 0;
    }
    for(int i=0; i<report_keyboard_length; i++) buffer_set(0x80+i, report_keyboard[i]);
    USB_BTABLE->ep[1].tx_count = report_keyboard_length;
    report_current = 2;
  }else{
    for(int i=0; i<report_volume_length; i++) buffer_set(0x80+i, report_volume[i]);
    USB_BTABLE->ep[1].tx_count = report_volume_length;
    report_current = 0;
  }
  */
}

void hid_init(void){
  usb_add_event_cb(event_cb);
  //register static setIdle() function somehow
  usb_add_descriptor((char *)&descriptor_interface);
  usb_add_descriptor((char *)&descriptor_hid);
  usb_add_descriptor((char *)&descriptor_endpoint);
  // usb_add_special_desciptor(USB_DESCRIPTOR_HID_REPORT, hid_report_descriptor, HID_REPORT_DESC_LEN); //special descriptor is not returned when getting configuration descriptor
  ep = usb_add_endpoint(USB_EP_OUT, 1, USB_EP_INTERRUPT,16);
  usb_add_endpoint_cb_tx(ep, cb_tx);
}
