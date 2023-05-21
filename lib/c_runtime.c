extern char _sidata, _sdata, _edata, _sbss, _ebss;
extern int main(void);

void _start(){
	char *src = &_sidata, *dst = &_sdata;
	while(dst < &_edata) *dst++ = *src++;
	for(dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
	main();
	while(1);
}
