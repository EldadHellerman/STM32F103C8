#include "usb.h"
#include "printf.h"

#define DEBUG 1 //note: printf waits for debugger, stalling the program if st-link isnt connected (stand alone usb device).
#define MAX_PACKET_SIZE0 32

#define USB_BUFFER_RX_COUNT_RECIEVED                  0x3FF << 0U
#define USB_BUFFER_RX_COUNT_BLOCKS                    0x1F << 0U
#define USB_BUFFER_RX_COUNT_BLOCK_SIZE                0x1 << 15U
#define USB_BUFFER_RX_GET_SIZE(x)                     ((x & USB_BUFFER_RX_COUNT_BLOCKS) * ((x & USB_BUFFER_RX_COUNT_BLOCK_SIZE) ? 32 : 2))
#define get_byte_from_struct(x,i)       (*((char *)(&x) + i))
#define MIN(a,b) ((a <= b) ? a : b)

struct usb_state{
  uint8_t address_change_pending : 1; //address change pending, waiting to send ack
  uint8_t address : 7; //new address to change to , or current addres if not pending
  uint8_t suspended : 1; //true if suspended
  uint8_t confiugration_active : 7; //active configuration, 0 is default
  //struct endpoint_transaction_state[16];
  usb_event_cb event_cb[10]; //up to 10 event_listeners
  char *descriptors[20]; //up to 20 descriptors
  usb_ep endpoints[16]; //up to 16 endpoints
  usb_cb_tx cb_tx[8];
  usb_cb_rx cb_rx[8];
} static state = {0};
//TODO add a configuration struct with number of interfaces etc.. , then have an array of possible configuration in usb state struct

char *strings[] = {"EH", "STM32 HID","HID configuration","mouse interface","00001"};
struct usb_descriptor_device descriptor_device = {18,USB_DESCRIPTOR_DEVICE,USB_1_1,0,0,0,MAX_PACKET_SIZE0,0xFFFF,0x0001,0x0100,1,2,5,1};
struct usb_descriptor_configuration descriptor_configuration = {9,USB_DESCRIPTOR_CONFIGURATION,34,1,1,0,0x80/*0xE0 for self powered and remote wakeup*/,50};

uint8_t buffer[34] = {0}; //TODO buffer is discarded if not initialized. check and change linker script!

usb_ep static ep0in, static ep0out;
//#define set_endpoint_buffer(int endpoint, int block_size, int blocks)
//#define BUF_read[i] (((USB_SRAM.buffer[i>>1]) >> ((i&1)*8)) & 0xff)
//#define BUF_read(i)  (USB_SRAM.buffer[i>>1] & 0xffff)
static void buffer_set(int i, char data){
  if(i & 0x01){
    USB_SRAM->buffer[i>>1] &= 0xffff00ff;
    USB_SRAM->buffer[i>>1] |= data << 8;
  }else{
    USB_SRAM->buffer[i>>1] &= 0xffffff00;
    USB_SRAM->buffer[i>>1] |= data;
  }
}

static char buffer_get(int i){
  if(i & 0x01) return((USB_SRAM->buffer[i>>1] & 0xff00) >> 8);
  return(USB_SRAM->buffer[i>>1] & 0xff);
}

void usb_memcpy_pma_to_sram(uint16_t pma_address, uint8_t *sram, uint16_t size){
  if(pma_address & 1){ //if starts at uneven addressing
    uint16_t data = USB_SRAM->buffer[pma_address>>1];
    *sram = data >> 8;
    sram++;
    pma_address++;
    size--;
  }
  while(size & (~1)){ //copy in half-words
    uint16_t data = USB_SRAM->buffer[pma_address>>1];
    *(sram) = data & 0xff;
    *(sram + 1) = data >> 8;
    size -= 2;
    sram += 2;
    pma_address += 2;
  }
  if(size){ //if there is a byte left (uneven transfer size)
    uint16_t data = USB_SRAM->buffer[pma_address>>1];
    *sram = data & 0xff;
    size--;
    sram++;
    pma_address++;
  }
}

void usb_memcpy_sram_to_pma(uint8_t *sram, uint16_t pma_address, uint16_t size){
  if(pma_address & 1){ //if copies to uneven addressing
    USB_SRAM->buffer[pma_address>>1] = ((USB_SRAM->buffer[pma_address>>1]) & 0xff) | *sram;
    sram++;
    pma_address++;
    size--;
  }
  while(size & (~1)){ //copy in half-words
    USB_SRAM->buffer[pma_address>>1] = *((uint16_t *)sram);
    //USB_SRAM->buffer[pma_address>>1] |= (*((uint16_t *)sram))<<8;
    size -= 2;
    sram += 2;
    pma_address += 2;
  }
  if(size){ //if there is a byte left (uneven transfer size)
    USB_SRAM->buffer[pma_address>>1] = (USB_SRAM->buffer[pma_address>>1] & 0xff00) | *sram;
    size--;
    sram++;
    pma_address++;
  }
}

////////////////////////////       debug functions      ///////////////////////////////////////

void delay(int time_us);

void signal(int i){
  i *= 2;
  while(i--) GPIOC->ODR ^= 1<<13;
}

///////////////////////////////////////////////////////////////////////////////////////////////

static void ep0_cb_rx(char *data, uint16_t size){
  //check if type is setup
  printf("ep0 Rx cb, data of size %d",size);
  if(USB->EP0R & USB_EP_SETUP) printf(", SETUP");
  printf("\n");
}

static void ep0_cb_tx(void){
  printf("ep0 Tx cb\n");
}

/*
at usb reset:
reset usb
init usb ep0 control communication via existing functions of adding descriptors and endpoints
call all reset event cb.
but then descriptors have to be added at each reset...
*/
void usb_init(void){
  RCC->APB1ENR |= RCC_APB1ENR_USBEN;
  RCC->APB1RSTR |= RCC_APB1RSTR_USBRST;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_USBRST;
  NVIC->ISER0 |= 0x100000; //USB LP
  USB->CNTR &= ~USB_CNTR_PDWN;
  for(volatile long int k=0; k<10000; k++); //delay
  for(int i=0; i<sizeof(state); i++) *(((uint8_t *)(&state)) + i) = 0;
  USB->CNTR = USB_CNTR_RESETM;
  USB->CNTR &= ~USB_CNTR_FRES;
}

void usb_reset_cb(void){
  //at usb reset all EPnR registers are reset. datasheet says DADDR reset's as well, but it doesnt - should be done manually.
  state.address_change_pending = 0;
  state.address = 0;
  state.suspended = 0;
  state.confiugration_active = 0;

  USB->DADDR = 0; //disable usb
  USB->EP0R = 0;
  USB->EP0R |= USB_EP_TYPE_CONTROL;
  USB->EP0R |= USB_EP_STAT_RX_VALID;// | USB_EP_STAT_TX_VALID;
  //may be in usb_init instead:
  for(int i=0;i<8;i++){
    USB_BTABLE->ep[i].tx_address = 0; USB_BTABLE->ep[i].tx_count = 0;
    USB_BTABLE->ep[i].rx_address = 0; USB_BTABLE->ep[i].rx_count = 0;
  }
  USB->BTABLE = 0x1C0;
  USB->ISTR = 0;
  for(int i=0; i<10; i++){
    if(state.event_cb[i] == 0) break;
    (*state.event_cb[i])(USB_EVENT_RESET);
  }
  ep0in = usb_add_endpoint(USB_EP_IN, 0, USB_EP_CONTROL, MAX_PACKET_SIZE0);
  ep0out = usb_add_endpoint(USB_EP_OUT, 0, USB_EP_CONTROL, MAX_PACKET_SIZE0);
  usb_add_endpoint_cb_rx(ep0in, usb_cb_rx cb)
  usb_add_endpoint_cb_tx(ep0out, usb_cb_tx cb)
  USB->CNTR |= USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
  USB->DADDR = USB_DADDR_EF; //enable usb
}

/*
void usb_send_zlp(){
  USB_BTABLE->ep[0].tx_count = 0;
  USB->EP0R = (USB->EP0R & 0x878f) | USB_EP_STAT_TX_0;
}

void usb_send_data_transaction(){
  if(state.tx_bytes_left){
    if(state.tx_bytes_left == MAX_PACKET_SIZE0) state.tx_need_to_send_zlp = 1;
    int send = (state.tx_bytes_left >= MAX_PACKET_SIZE0) ? MAX_PACKET_SIZE0 : state.tx_bytes_left;
    usb_memcpy_sram_to_pma(state.tx_next_data, USB_BTABLE->ep[0].tx_address, send);
    state.tx_next_data += send;
    state.tx_bytes_left -= send;
    USB_BTABLE->ep[0].tx_count = send;
    USB->EP0R = (USB->EP0R & 0x878f) | USB_EP_STAT_TX_0;
  }else if(state.tx_need_to_send_zlp){ //a zlp is needed
    state.tx_need_to_send_zlp = 0;
    usb_send_zlp();
  }
  if((state.tx_bytes_left == 0) && (state.tx_need_to_send_zlp==0)) state.tx_waiting_for_zlp = 1; //TODO maybe use hardware built-in status out bit.
}

void usb_send_data(uint8_t *data, uint16_t size){
  if(size == 0){
    state.tx_data = 0;
    state.tx_next_data = 0;
    state.tx_waiting_for_zlp = 0;
    state.tx_need_to_send_zlp = 1;
    state.tx_bytes_left = 0;
  }else{
    state.tx_data = data;
    state.tx_next_data = data;
    state.tx_waiting_for_zlp = 0;
    state.tx_need_to_send_zlp = 0;
    state.tx_bytes_left = size;
  }
  usb_send_data_transaction();
}

void usb_setup_cb(){
  struct usb_setup_request_pma *p = (struct usb_setup_request_pma *)(USB_SRAM_BASE + (USB_BTABLE->ep[0].rx_address << 1));
  // if(DEBUG) printf("setup request, type: 0x%.2X, request: 0x%.2X, value: 0x%.4X, index: 0x%.4X, length: 0x%.4X\n", p->bmRequestType, p->bRequest, p->wValue, p->wIndex, p->wLength);
  if((p->bmRequestType & (USB_SETUP_REQUESTTYPE_RECIPIENT | USB_SETUP_REQUESTTYPE_TYPE)) == 0x21){ //class request to interface
    if(DEBUG) printf("special class Transfer ...\n");
    if(p->bRequest == 0x0A){ //set idle
      if(DEBUG) printf("HID set idle request!\n");
      signal(8);
      usb_send_zlp();
    }
    return;
  }
  signal(3);
  if(p->bRequest == USB_SETUP_REQUEST_GET_DESCRIPTOR){
    // go through descriptors[20] to see if theres a descriptor matching
    int maxLength = p->wLength;
    if(p->wValue_h == 0x01){ //DEVICE
      signal(1);
      usb_send_data((uint8_t *)&descriptor_device, MIN(descriptor_device.bLength, maxLength));
    }else if(p->wValue_h == 0x02){ //CONFIGURATION
      signal(2);
      for(int i=0; i<9; i++) buffer[i] = get_byte_from_struct(descriptor_configuration,i);
      for(int i=0; i<9; i++) buffer[i + 9] = get_byte_from_struct(descriptor_interface,i);
      for(int i=0; i<9; i++) buffer[i + 18] = descriptor_hid[i];
      for(int i=0; i<7; i++) buffer[i + 27] = get_byte_from_struct(descriptor_endpoint,i);
      usb_send_data(buffer, MIN(34, maxLength));
    }else if(p->wValue_h == 0x03){ //STRING
      if(p->wValue_l == 0){ //INDEX 0 - list of supported languages
        uint8_t buffer[4] = {4,0x03,0x09,0x04}; //us supported only
        usb_send_data(buffer, MIN(4, maxLength));
      }else{
        //TODO go through string once then update length byte (instead of going twice)
        char *s = strings[p->wValue_l-1];
        int length = 0;
        while(*(s+length)) length++;
        if(length > 15) length = 15;
        short buffer[32];
        length = (length*2 + 2);
        buffer[0] = (0x03<<8) | length;
        int i=1;
        while(*s) buffer[i++] = *s++;
        usb_send_data((uint8_t *)buffer, length);
      }
    }else if(p->wValue_h == 0x21){ //HID class HID descriptor
    }else if(p->wValue_h == 0x22){ //HID class report descriptor
      if(DEBUG) printf("get report request\n");
      //usb_send_data(HID_report_descriptor, MIN(50,maxLength));
      usb_send_data(HID_report_descriptor, MIN(HID_report_descriptor_length,maxLength));
    }else if(p->wValue_h == 0x23){ //HID class physical descriptor
    }
  }else if(p->bRequest == USB_SETUP_REQUEST_SET_ADDRESS){ //set address
    state.address = p->wValue_l & 0x7f;
    state.address_change_pending = 1; //do not assign address before status transaction.
    usb_send_zlp();
  }else if(p->bRequest == USB_SETUP_REQUEST_SET_CONFIGURATION){ //set configuration
    state.configuration = p->wValue_l;
    usb_send_zlp();
  }
}

void usb_crt_cb(int endpoint){
  signal(1);

  // if dir == tx{
  //   //go throug state.cb_tx
  // }else{
  //   //go throug state.cb_rx
  // }

  if(endpoint == 0){
    if(USB->EP0R & USB_EP_CTR_RX){
      if(USB->EP0R & USB_EP_SETUP){
        usb_setup_cb();
      }else{
        int size = USB_BTABLE->ep[0].rx_count & USB_BUFFER_RX_COUNT_RECIEVED;
        if((size == 0) && state.tx_waiting_for_zlp){
          state.tx_waiting_for_zlp = 0;
        }else{
          signal(2);
          if(DEBUG) printf("recived data of size %d:    ", size);
          for(int i=0; i<size; i++) if(DEBUG) printf("%.2X, ", buffer_get(USB_BTABLE->ep[0].rx_address + i));
          if(DEBUG) printf("\n");
        }
      }
      USB_BTABLE->ep[0].rx_count &= ~0x3ff; //zero's Rx byte count
      USB->EP0R = (USB->EP0R & 0x878f) & ~0x8000; //clear CTR_RX
      USB->EP0R = (USB->EP0R & 0x878f) | USB_EP_STAT_RX_0; //set RX as VALID
    }
    if(USB->EP0R & USB_EP_CTR_TX){
      if(state.address_change_pending){ //if after set address, and a ZLP is required as an ack
        state.address_change_pending = 0;
        USB->DADDR = USB_DADDR_EF | state.address;
        signal(6);
      }
      usb_send_data_transaction();
      USB->EP0R = (USB->EP0R & 0x878f) & ~0x80; //clear CTR_TX
    }
  }else if(endpoint == 1){
    // if(DEBUG) printf("ep1!\n");
    if(USB->EP1R & USB_EP_CTR_RX){
      USB->EP1R = (USB->EP1R & 0x878f) & ~0x8000; //clear CTR_RX
      USB->EP1R = (USB->EP1R & 0x878f) | USB_EP_STAT_RX_0; //set RX as VALID
    }else if(USB->EP1R & USB_EP_CTR_TX){

      USB->EP1R = (USB->EP1R & 0x878f) & ~0x80; //clear CTR_TX
      USB->EP1R = (USB->EP1R & 0x878f) | USB_EP_STAT_TX_0; //set TX as VALID
    }
  }
}
*/
void _IRQ20_USB_LP_CAN_RX0(void){
  GPIOC->ODR ^= 1<<13;
  if(DEBUG){
    printf("USB interrupt - ");
    // printf("USB #%.2X  -  ", packet_number);
    if(USB->ISTR & USB_ISTR_RESET) printf("reset, ");
    if(USB->ISTR & USB_ISTR_SUSP) printf("suspend, ");
    if(USB->ISTR & USB_ISTR_WKUP) printf("wakeup, ");
    if(USB->ISTR & USB_ISTR_SOF) printf("sof, ");
    if(USB->ISTR & USB_ISTR_CTR) printf("CT, ");
    if(USB->ISTR & USB_ISTR_PMAOVR) printf("PMAO, ");
    if(USB->ISTR & USB_ISTR_ERR) printf("error, ");
    printf("\n");
  }
  if(USB->ISTR & USB_ISTR_CTR){
    // usb_crt_cb(USB->ISTR & USB_ISTR_EP_ID);
    int i = 0;
    while(state.endpoints[i].RESERVED){
      if((state.endpoints[i].address == USB->ISTR & USB_ISTR_EP_ID) && state.endpoints[i].direction != USB_ISTR_DIR){ //ISTR_DIR uses opposite convention from EPnR direction
        printf("ctr matching existing cb!\n");
        if(USB_ISTR_DIR == 0){ //OUT
          *(state.cb_tx[state.endpoints[i].number])();
        }else{ //IN
          *(state.cb_rx[state.endpoints[i].number])();
        }
      }
      i++;
    }

    state.cb_tx[ep.number]
  }else if(USB->ISTR & USB_ISTR_RESET){
    signal(2);
    usb_reset_cb();
  }else if(USB->ISTR & USB_ISTR_SUSP){
    signal(10);
    USB->CNTR |= USB_CNTR_FSUSP;
  }else if(USB->ISTR & USB_ISTR_WKUP){
    signal(15);
    USB->CNTR &= ~USB_CNTR_FSUSP;
  }else{
    signal(5);
  }
  USB->ISTR = 0;
  GPIOC->ODR ^= 1<<13;
}

void usb_add_event_cb(usb_event_cb cb){
  if(DEBUG) printf("usb add event_cb\n");
  int i=0;
  while(state.event_cb[i] != 0) i++;
  state.event_cb[i] = cb;
};

void usb_add_descriptor(char *desc){
  if(DEBUG) printf("usb add descriptor\n");
  int i=0;
  while(state.descriptors[i] != 0) i++;
  state.descriptors[i] = desc;
}

//void usb_add_special_descriptor(uint16_t length, uint8_t type, char *desc); //not included when sending configuration description

usb_ep usb_add_endpoint(uint8_t direction, uint8_t address, uint8_t type, uint16_t maxPacketSize){
  if(DEBUG) printf("usb add ep\n");
  int i=0;
  while(state.endpoints[i].RESERVED) i++;
  if(i < 2) i = 2; //TODO remove this when usb itself adds endpoint 0 using addendpoint function
  state.endpoints[i].RESERVED = 1;
  state.endpoints[i].direction = direction;
  state.endpoints[i].number = i;
  state.endpoints[i].address = address;
  if(direction == USB_EP_IN){
    USB_BTABLE->ep[i].rx_address = USB_BTABLE->ep[i-1].rx_address + USB_BUFFER_RX_GET_SIZE(USB_BTABLE->ep[i-1].rx_count);
    USB_BTABLE->ep[i].rx_count = (maxPacketSize / 2) << 10;
  }else{
    USB_BTABLE->ep[i].tx_address = USB_BTABLE->ep[i-1].tx_address + USB_BTABLE->ep[i-1].tx_count;
    USB_BTABLE->ep[i].tx_count = maxPacketSize;
  }
  switch(i){
    case 0: USB->EP0R = (type << 9) | address; break;
    case 1: USB->EP1R = (type << 9) | address; break;
    case 2: USB->EP2R = (type << 9) | address; break;
    case 3: USB->EP3R = (type << 9) | address; break;
    case 4: USB->EP4R = (type << 9) | address; break;
    case 5: USB->EP5R = (type << 9) | address; break;
    case 6: USB->EP6R = (type << 9) | address; break;
    case 7: USB->EP7R = (type << 9) | address; break;
  }
  return(state.endpoints[i]);
}

void usb_endpoint_set_status(usb_ep ep, int status){
  if(DEBUG) printf("usb set ep status\n");
  if(ep.direction == USB_EP_IN){
    switch(ep.number){
      case 0:
        if((((USB->EP0R & 0x878f) & USB_EP_STAT_RX_0) >> 12) != (status & 0x01)) USB->EP0R = (USB->EP0R & 0x878f) | USB_EP_STAT_RX_0; //flip bit 0
        if((((USB->EP0R & 0x878f) & USB_EP_STAT_RX_1) >> 12) != (status & 0x02)) USB->EP0R = (USB->EP0R & 0x878f) | USB_EP_STAT_RX_1; //flip bit 1
        break;
      case 1:
        if((((USB->EP1R & 0x878f) & USB_EP_STAT_RX_0) >> 12) != (status & 0x01)) USB->EP1R = (USB->EP1R & 0x878f) | USB_EP_STAT_RX_0; //flip bit 0
        if((((USB->EP1R & 0x878f) & USB_EP_STAT_RX_1) >> 12) != (status & 0x02)) USB->EP1R = (USB->EP1R & 0x878f) | USB_EP_STAT_RX_1; //flip bit 1
        break;
      case 2:
        if((((USB->EP2R & 0x878f) & USB_EP_STAT_RX_0) >> 12) != (status & 0x01)) USB->EP3R = (USB->EP2R & 0x878f) | USB_EP_STAT_RX_0; //flip bit 0
        if((((USB->EP2R & 0x878f) & USB_EP_STAT_RX_1) >> 12) != (status & 0x02)) USB->EP3R = (USB->EP2R & 0x878f) | USB_EP_STAT_RX_1; //flip bit 1
        break;
      case 3:
        if((((USB->EP3R & 0x878f) & USB_EP_STAT_RX_0) >> 12) != (status & 0x01)) USB->EP3R = (USB->EP3R & 0x878f) | USB_EP_STAT_RX_0; //flip bit 0
        if((((USB->EP3R & 0x878f) & USB_EP_STAT_RX_1) >> 12) != (status & 0x02)) USB->EP3R = (USB->EP3R & 0x878f) | USB_EP_STAT_RX_1; //flip bit 1
        break;
      case 4:
        if((((USB->EP4R & 0x878f) & USB_EP_STAT_RX_0) >> 12) != (status & 0x01)) USB->EP4R = (USB->EP4R & 0x878f) | USB_EP_STAT_RX_0; //flip bit 0
        if((((USB->EP4R & 0x878f) & USB_EP_STAT_RX_1) >> 12) != (status & 0x02)) USB->EP4R = (USB->EP4R & 0x878f) | USB_EP_STAT_RX_1; //flip bit 1
        break;
      case 5:
        if((((USB->EP5R & 0x878f) & USB_EP_STAT_RX_0) >> 12) != (status & 0x01)) USB->EP5R = (USB->EP5R & 0x878f) | USB_EP_STAT_RX_0; //flip bit 0
        if((((USB->EP5R & 0x878f) & USB_EP_STAT_RX_1) >> 12) != (status & 0x02)) USB->EP5R = (USB->EP5R & 0x878f) | USB_EP_STAT_RX_1; //flip bit 1
        break;
      case 6:
        if((((USB->EP6R & 0x878f) & USB_EP_STAT_RX_0) >> 12) != (status & 0x01)) USB->EP6R = (USB->EP6R & 0x878f) | USB_EP_STAT_RX_0; //flip bit 0
        if((((USB->EP6R & 0x878f) & USB_EP_STAT_RX_1) >> 12) != (status & 0x02)) USB->EP6R = (USB->EP6R & 0x878f) | USB_EP_STAT_RX_1; //flip bit 1
        break;
      case 7:
        if((((USB->EP7R & 0x878f) & USB_EP_STAT_RX_0) >> 12) != (status & 0x01)) USB->EP7R = (USB->EP7R & 0x878f) | USB_EP_STAT_RX_0; //flip bit 0
        if((((USB->EP7R & 0x878f) & USB_EP_STAT_RX_1) >> 12) != (status & 0x02)) USB->EP7R = (USB->EP7R & 0x878f) | USB_EP_STAT_RX_1; //flip bit 1
        break;
    }
  }else{
    switch(ep.number){
      case 0:
        if((((USB->EP0R & 0x878f) & USB_EP_STAT_TX_0) >> 4) != (status & 0x01)) USB->EP0R = (USB->EP0R & 0x878f) | USB_EP_STAT_TX_0; //flip bit 0
        if((((USB->EP0R & 0x878f) & USB_EP_STAT_TX_1) >> 4) != (status & 0x02)) USB->EP0R = (USB->EP0R & 0x878f) | USB_EP_STAT_TX_1; //flip bit 1
        break;
      case 1:
        if((((USB->EP1R & 0x878f) & USB_EP_STAT_TX_0) >> 4) != (status & 0x01)) USB->EP1R = (USB->EP1R & 0x878f) | USB_EP_STAT_TX_0; //flip bit 0
        if((((USB->EP1R & 0x878f) & USB_EP_STAT_TX_1) >> 4) != (status & 0x02)) USB->EP1R = (USB->EP1R & 0x878f) | USB_EP_STAT_TX_1; //flip bit 1
        break;
      case 2:
        if((((USB->EP2R & 0x878f) & USB_EP_STAT_TX_0) >> 4) != (status & 0x01)) USB->EP3R = (USB->EP2R & 0x878f) | USB_EP_STAT_TX_0; //flip bit 0
        if((((USB->EP2R & 0x878f) & USB_EP_STAT_TX_1) >> 4) != (status & 0x02)) USB->EP3R = (USB->EP2R & 0x878f) | USB_EP_STAT_TX_1; //flip bit 1
        break;
      case 3:
        if((((USB->EP3R & 0x878f) & USB_EP_STAT_TX_0) >> 4) != (status & 0x01)) USB->EP3R = (USB->EP3R & 0x878f) | USB_EP_STAT_TX_0; //flip bit 0
        if((((USB->EP3R & 0x878f) & USB_EP_STAT_TX_1) >> 4) != (status & 0x02)) USB->EP3R = (USB->EP3R & 0x878f) | USB_EP_STAT_TX_1; //flip bit 1
        break;
      case 4:
        if((((USB->EP4R & 0x878f) & USB_EP_STAT_TX_0) >> 4) != (status & 0x01)) USB->EP4R = (USB->EP4R & 0x878f) | USB_EP_STAT_TX_0; //flip bit 0
        if((((USB->EP4R & 0x878f) & USB_EP_STAT_TX_1) >> 4) != (status & 0x02)) USB->EP4R = (USB->EP4R & 0x878f) | USB_EP_STAT_TX_1; //flip bit 1
        break;
      case 5:
        if((((USB->EP5R & 0x878f) & USB_EP_STAT_TX_0) >> 4) != (status & 0x01)) USB->EP5R = (USB->EP5R & 0x878f) | USB_EP_STAT_TX_0; //flip bit 0
        if((((USB->EP5R & 0x878f) & USB_EP_STAT_TX_1) >> 4) != (status & 0x02)) USB->EP5R = (USB->EP5R & 0x878f) | USB_EP_STAT_TX_1; //flip bit 1
        break;
      case 6:
        if((((USB->EP6R & 0x878f) & USB_EP_STAT_TX_0) >> 4) != (status & 0x01)) USB->EP6R = (USB->EP6R & 0x878f) | USB_EP_STAT_TX_0; //flip bit 0
        if((((USB->EP6R & 0x878f) & USB_EP_STAT_TX_1) >> 4) != (status & 0x02)) USB->EP6R = (USB->EP6R & 0x878f) | USB_EP_STAT_TX_1; //flip bit 1
        break;
      case 7:
        if((((USB->EP7R & 0x878f) & USB_EP_STAT_TX_0) >> 4) != (status & 0x01)) USB->EP7R = (USB->EP7R & 0x878f) | USB_EP_STAT_TX_0; //flip bit 0
        if((((USB->EP7R & 0x878f) & USB_EP_STAT_TX_1) >> 4) != (status & 0x02)) USB->EP7R = (USB->EP7R & 0x878f) | USB_EP_STAT_TX_1; //flip bit 1
        break;
    }
  }
}

void usb_add_endpoint_cb_tx(usb_ep ep, usb_cb_tx cb){
  if(DEBUG) printf("usb add ep cb tx\n");
  state.cb_tx[ep.number] = cb;
}

void usb_add_endpoint_cb_rx(usb_ep ep, usb_cb_rx cb){
  if(DEBUG) printf("usb add ep cb rx\n");
  state.cb_rx[ep.number] = cb;
}

void usb_send(usb_ep ep, uint8_t *data, uint16_t size){
  USB_BTABLE->ep[ep.number].tx_count = size;
  usb_memcpy_sram_to_pma(data,USB_BTABLE->ep[ep.number].tx_address, size);
  usb_endpoint_set_status(ep, USB_EP_VALID);
}


void usb_print_status(void){
  printf("printing status\n");
  int i = 0;
  printf("    event_cb: "); for(int i=0; (i<10) && (state.event_cb[i] != 0); i++){ printf("%p,",state.event_cb[i]);} printf("\n");
  printf("    descriptors: "); for(int i=0; (i<20) && (state.descriptors[i] != 0); i++){ printf("%p,",state.descriptors[i]);} printf("\n");
  printf("    cb_tx: "); for(int i=0; (i<8) && (state.cb_tx[i] != 0); i++){ printf("%p,",state.cb_tx[i]);} printf("\n");
  printf("    cb_rx: "); for(int i=0; (i<8) && (state.cb_rx[i] != 0); i++){ printf("%p,",state.cb_rx[i]);} printf("\n");
  printf("    endpoints: ");
  for(int i=0; (i<16) && (state.endpoints[i].RESERVED != 0); i++){
    printf("ep number %d:", state.endpoints[i].number);
    printf("        direction: %s\n", (state.endpoints[i].direction == USB_EP_IN) ? "IN" : "OUT");
  }
  printf("\n");
}
