#ifndef DEFS_H
#define DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#define ESUCCESS 0
#define PORT 4444
#define CHECK(B) do { if (!(B)) { \
	if( errno == ESUCCESS){ \
printf("Some Error occured\n"); \
exit(1); \
} \
else { \
perror("Error occured"); \
exit(1); \
}}}while (0)

inline void printhex(const char *data, size_t len) {
size_t i;
for(i = 0; i < len; i++)
printf("%02hhx ", data[i]);
printf("\n");
}

#ifdef __cplusplus
}
#endif

#endif
