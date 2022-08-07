
#include <stdio.h>
#include <stdlib.h>





void memcpy(void*dest,const void * src, size_t n);

int main ()
{

int rapdu[]= {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};

int pan[9]={0,0,0,0,0,0,0,0,0};

memcpy(pan,rapdu+5,8*4);


for(int i=0; i<8;i++)
printf("%d ",pan[i]);

}

void memcpy(void *dest,const void *src, size_t n)
{
	char *tmp = dest;
	const char *s = (char*)src;
	while (n--)
	*tmp++ = *s++;	
}



