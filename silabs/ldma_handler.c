
#include "platform.h"
#include "em_device.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "ldma_handler.h"
#include "em_ldma.h"
#include "cmsis_os2.h"
#include "sys_panic.h"

#define SIZE_OF_ARRAY(arr) (sizeof(arr))/sizeof(arr[0])

static volatile ldma_handler_conf_t m_head;

void LDMA_IRQHandler (void)
{
	uint32_t pending = LDMA_IntGet();
    PLATFORM_LedsSet(PLATFORM_LedsGet()^1);
	uint32_t error = 0;
    while (pending & LDMA_IF_ERROR)
	{
		//err1("ldma if");
	}

	LDMA_IntClear(pending);

    ldma_handler_conf_t* ptr = &m_head;
    while(ptr != NULL)
    {
        if ( pending & (1<<ptr->channel) )
	    {
		    if(ptr->name == 69)
            {
                error = osThreadFlagsSet(ptr->thrd, ptr->signal);

            }
	    }
        ptr = ptr->next;
    }
}

void append_to_ldma_stored_configuration(ldma_handler_conf_t * newconf)
{
    //append to configuration
    ldma_handler_conf_t *ptr = &m_head;
    if(!m_head.next)
    {
        // adding first config
        ptr->next = newconf;
    }
    else
    {
       //looping to the end of list
        while(ptr != NULL && ptr->next != NULL)
        {
            ptr = ptr->next;
        }
        //adding new element
        ptr->next = newconf;
    }
}
