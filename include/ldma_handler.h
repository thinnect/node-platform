#ifndef LDMA_HANDLER_CONF
#define LDMA_HANDLER_CONF

#include "cmsis_os2.h"

typedef struct {
    int channel;
    osThreadId_t thrd;
    int name;
    int signal;
    struct ldma_handler_conf_t* next;
} ldma_handler_conf_t;

void append_to_ldma_stored_configuration(ldma_handler_conf_t * newconf);

#endif //LDMA_HANDLER_CONF
