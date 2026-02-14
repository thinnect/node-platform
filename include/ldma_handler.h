#ifndef LDMA_HANDLER_CONF
#define LDMA_HANDLER_CONF

#include "cmsis_os2.h"

typedef struct {
    uint32_t channel;
    osThreadId_t thrd;
    uint32_t name;
    uint32_t signal;
    struct ldma_handler_conf_t* next;
} ldma_handler_conf_t;

void append_to_ldma_stored_configuration(ldma_handler_conf_t * newconf);

#endif //LDMA_HANDLER_CONF
