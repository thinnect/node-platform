#include "radio.h"
#include "bus_dev.h"
#include "sys_panic.h"
#include "assert.h"

#define RADIO_MAX_SEND_TIME_MS 10000UL

// Thread flag definitions
#define RDFLG_RADIO_DEINIT        (1 << 0)
#define RDFLG_RADIO_START         (1 << 1)
#define RDFLG_RADIO_STOP          (1 << 2)
#define RDFLG_RADIO_RESTART       (1 << 3)
#define RDFLG_RADIO_SEND          (1 << 4)
#define RDFLG_RADIO_RESEND        (1 << 5)
#define RDFLG_RADIO_SEND_TIMEOUT  (1 << 6)
#define RDFLG_RADIO_SEND_FAIL     (1 << 7)

#define RDFLG_RAIL_SEND_DONE      (1 << 8)
#define RDFLG_RAIL_SEND_BUSY      (1 << 9)
#define RDFLG_RAIL_SEND_FAIL      (1 << 10)
#define RDFLG_RAIL_RX_BUSY        (1 << 11)
#define RDFLG_RAIL_RX_SUCCESS     (1 << 12)
#define RDFLG_RAIL_RX_OVERFLOW    (1 << 13)
#define RDFLG_RAIL_RX_FRAME_ERROR (1 << 14)
#define RDFLG_RAIL_RX_ABORT       (1 << 15)
#define RDFLG_RAIL_RX_FAIL        (1 << 16)
#define RDFLG_RAIL_TXACK_SENT     (1 << 17)
#define RDFLG_RAIL_RXACK_TIMEOUT  (1 << 18)
#define RDFLG_RAIL_RX_MORE        (1 << 20)

#define RDFLGS_ALL                (0x7FFFFFFF)

#define LL_HW_MODE_STX                  0x00
#define LL_HW_MODE_SRX                  0x01
#define LL_HW_MODE_TRX                  0x02
#define LL_HW_MODE_RTX                  0x03
#define LL_HW_MODE_TRLP                 0x04
#define LL_HW_MODE_RTLP                 0x05

// queue -----------------------------------------------------------------------
typedef struct radio_queue_element radio_queue_element_t;
struct radio_queue_element
{
    comms_msg_t* msg;
    comms_send_done_f *send_done;
    void *user;
    uint32_t timestamp_queued;
    radio_queue_element_t* next;
};
// -----------------------------------------------------------------------------

// TX queue with a linked list -------------------------------------------------
static radio_queue_element_t radio_msg_queue_memory[7];
static radio_queue_element_t* radio_msg_queue_free = NULL;
static radio_queue_element_t* radio_msg_queue_head = NULL;
static radio_queue_element_t* radio_msg_sending  = NULL;
// -----------------------------------------------------------------------------


typedef enum RadioState
{
    ST_UNINITIALIZED,
    ST_OFF,
    ST_STARTING,
    ST_RUNNING,
    ST_STOPPING
} RadioState_t;



extern void hal_rom_boot_init(void);

static uint8_t m_rxBuf[127] ={0};
static uint16_t volatile irqflag = 0;
static uint32_t m_foot[2] = {0};
static uint8_t  m_packet_len = 0;
static uint16_t m_pktLen = 0;
static comms_layer_am_t m_radio_iface;
static volatile uint8_t rx_busy;
static volatile uint8_t rx_overflow;
static volatile uint8_t rx_frame_error;
static volatile uint8_t rx_abort;
static volatile uint8_t rx_fail;
static volatile uint8_t tx_ack_sent;

static osMutexId_t m_radio_mutex;

static radio_config_t m_config = {0};
static uint8_t m_radio_tx_num;
static bool radio_tx_wait_ack;
static RadioState_t m_state = ST_UNINITIALIZED;

static osTimerId_t m_send_timeout_timer;
static osTimerId_t m_resend_timer;

static uint32_t m_radio_send_timestamp;


static void zb_hw_go(void)
{
    *(volatile uint32_t *)(LL_HW_BASE+ 0x14) = LL_HW_IRQ_MASK;    //clr  irq status
    *(volatile uint32_t *)(LL_HW_BASE+ 0x0c) = 0x0001;            //mask irq :only use mode done
    *(volatile uint32_t *)(LL_HW_BASE+ 0x00) = 0x0001;            //trig

    uint8_t rfChnIdx = PHY_REG_RD(0x400300b4)&0xff;


    if(rfChnIdx<2)
    {
        rfChnIdx=2;
    }
    else if(rfChnIdx>80)
    {
        rfChnIdx=80;
    }
		rf_tpCal_cfg(rfChnIdx);

#if (DBG_BUILD_LL_TIMING)
    hal_gpio_write(DBG_PIN_LL_HW_TRIG, 1);
    hal_gpio_write(DBG_PIN_LL_HW_TRIG, 0);
#endif
}

static void zb_hw_stop(void)
{
    subWriteReg(AP_PCR_BASE+0x0c,10,10,0);//reset bbll

    PHY_REG_WT( 0x400300a4,0x00000140);     // clr tx_auto
    PHY_REG_WT( 0x400300a0,0x0000000e);     // clr pll_auto override
    PHY_REG_WT( 0x400300a0,0x00000000);     // clr pll_auto override

    subWriteReg(AP_PCR_BASE+0x0c,10,10,1);//release bbll reset

    //restore the ll register
    ll_hw_set_tx_rx_release	(10,   	 1);
    ll_hw_set_rx_tx_interval(    	98);		//T_IFS=192+2us for ZB
    ll_hw_set_tx_rx_interval(      108);		//T_IFS=192-6us for ZB

}

static void zb_hw_set_srx(uint32_t rxTimeOutUs)
{
    ll_hw_set_rx_timeout(rxTimeOutUs);
    ll_hw_set_srx();
    ll_hw_set_trx_settle(32, 8, 52);          //RxAFE,PLL

		m_config.mode = RX_ONLY;
}

static void zb_hw_set_stx(void)
{
    ll_hw_set_stx();
    ll_hw_set_trx_settle(32, 8, 52);          //RxAFE,PLL

		m_config.mode = TX_ONLY;
}

/*
static void zb_hw_set_trx(uint32_t rxTimeOutUs)
{
    ll_hw_set_rx_timeout(rxTimeOutUs);
    ll_hw_set_trx();
    ll_hw_set_trx_settle(32, 8, 52);          //RxAFE,PLL
		m_config.mode = TX_RX_MODE;
}
*/

static void zb_set_channel(uint8_t chn)
{
    uint32_t rfChnIdx = (chn - 10) * 5;

    if(g_rfPhyFreqOffSet>=0)
        PHY_REG_WT(0x400300b4, (g_rfPhyFreqOffSet<<16)+(g_rfPhyFreqOffSet<<8)+rfChnIdx);
    else
        PHY_REG_WT(0x400300b4, ((255+g_rfPhyFreqOffSet)<<16)+((255+g_rfPhyFreqOffSet)<<8)+(rfChnIdx-1) );
    ll_hw_ign_rfifo(LL_HW_IGN_CRC);
}


static uint8_t  zbll_hw_read_rfifo_zb(uint8_t* rxPkt, uint16_t* pktLen, uint32_t* pktFoot0, uint32_t* pktFoot1)
{
    int rdPtr,wrPtr,rdDepth,blen,wlen;
    uint32_t* p_rxPkt=(uint32_t *)rxPkt;

    ll_hw_get_rfifo_info(&rdPtr,&wrPtr,&rdDepth);

    if(rdDepth>0){

        *p_rxPkt++ = *(volatile uint32_t *)(LL_HW_RFIFO);

        blen      = rxPkt[0];                        //get the byte length for header
        wlen    = 1+ ( (blen) >>2 );             //blen included the 2byte crc

        while(p_rxPkt<(uint32_t *)rxPkt+wlen){
            *p_rxPkt++ = *(volatile uint32_t *)(LL_HW_RFIFO);
        }

        *pktFoot0     = *(volatile uint32_t *)(LL_HW_RFIFO);
        *pktFoot1     = *(volatile uint32_t *)(LL_HW_RFIFO);

        *pktLen     = blen+1;


        return wlen;

    }else{

        rxPkt[0]  = 0;
        *pktFoot0 = 0;
        *pktFoot1 = 0;
        *pktLen   = 0;
        return 0;
    }



}

static uint32_t radio_timestamp ()
{
    return read_current_fine_time();
}

void phy_rf_rx(void)
{
    zb_hw_stop();
    HAL_ENTER_CRITICAL_SECTION();

    zb_hw_set_srx(0);

    ll_hw_rst_tfifo();
    ll_hw_rst_rfifo();
    set_max_length(0xff);
    
    zb_hw_go();
    //llWaitingIrq=TRUE;
    HAL_EXIT_CRITICAL_SECTION();
}

void ZBRFPHY_IRQHandler(void)
{
	 uint8_t buf[140] = {0};
	 uint32_t rts = radio_timestamp();
	 irqflag = ll_hw_get_irq_status();
	 HAL_ENTER_CRITICAL_SECTION();
				if(irqflag & LIRQ_MD)
				{
					if(irqflag & LIRQ_COK)
					{
						if(m_config.mode == RX_ONLY || m_config.mode == TX_RX_MODE)
						{
								m_packet_len = zbll_hw_read_rfifo_zb(&buf[0], &m_pktLen, &m_foot[0], &m_foot[1]);
								if(m_packet_len>0)
								{

										mac_frame_t* frame = (mac_frame_t*)&buf[1];
										if((frame->frame_control[1] & MAC_FCF_DST_ADDR_BIT) == 0x08)
										{
											if(frame->dst == m_config.nodeaddr || frame->dst == BROADCAST_ADDR)
											{
												// Receive timestamp, added to message buffer
												uint8_t len = buf[0];
												buf[len] = rts >> 24;
												buf[len + 1] = rts >> 16;
												buf[len + 2] = rts >> 8;
												buf[len + 3] = rts;
												
												int res = osMessageQueuePut(m_config.recvQueue, &buf, 0, 0); //TODO packet len off by 2
												if(osOK != res)
												{
													LOG("Failed to put message in queue %i\r\n", res);
												}
												else
												{
													osThreadFlagsSet(m_config.threadid, RDFLG_RAIL_RX_SUCCESS);
													phy_rf_rx();
												}
										  }
									 }
								}
						}
					}


					if( (irqflag & LIRQ_TD) && m_config.mode == TX_ONLY)
					{
						 osThreadFlagsSet(m_config.threadid, RDFLG_RAIL_SEND_DONE);
						 LOG("Switching back to RX\r\n");
						 m_config.mode = RX_ONLY;
						phy_rf_rx();

					}

				}

					if(irqflag & LIRQ_CERR)
					{
						LOG("Error occured");
					}
					if(irqflag & LIRQ_CERR2)
					{
						LOG("Error Occured2");
					}
					if(irqflag & LIRQ_RTO)
					{
						LOG("RX Timed out");
					}




			ll_hw_clr_irq();

		 HAL_EXIT_CRITICAL_SECTION();
}

comms_error_t radio_send(comms_layer_iface_t* interface, comms_msg_t* msg, comms_send_done_f* cb, void* user/*uint8_t* data, uint16_t len, uint16_t dst*/)
{

		comms_error_t err = COMMS_FAIL;

    if (interface != (comms_layer_iface_t *)&m_radio_iface)
    {
        return(COMMS_EINVAL);
    }

    while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever));

    if (ST_RUNNING != m_state)
    {
        err1("radio off");
        err = COMMS_EOFF;
    }
    else if (radio_msg_queue_free != NULL)
    {
        radio_queue_element_t* qm = (radio_queue_element_t*)radio_msg_queue_free;
        radio_msg_queue_free = radio_msg_queue_free->next;

        qm->msg = msg;
        qm->send_done = cb;
        qm->user = user;
        qm->timestamp_queued = radio_timestamp();
        qm->next = NULL;

        if (radio_msg_queue_head == NULL)
        {
            radio_msg_queue_head = qm;
        }
        else
        {
            radio_queue_element_t* qn = (radio_queue_element_t*)radio_msg_queue_head;
            while (qn->next != NULL)
            {
                qn = qn->next;
            }
            qn->next = qm;
        }

        osThreadFlagsSet(m_config.threadid, RDFLG_RADIO_SEND);

//        info3("snd %p \r\n", msg);
        err = COMMS_SUCCESS;
    }
    else
    {
        warn1("busy");
        err = COMMS_ENOMEM;
    }

    osMutexRelease(m_radio_mutex);

    return err;
}

/*
static void rf_wakeup_handler(void){
  NVIC_SetPriority((IRQn_Type)4, IRQ_PRIO_REALTIME);
  NVIC_SetPriority((IRQn_Type)20, IRQ_PRIO_HIGH);
}
*/

static void hal_rfphy_init(void)
{
    //============config the txPower
    g_rfPhyTxPower  = RF_PHY_TX_POWER_EXTRA_MAX;
    //============config BLE_PHY TYPE
    g_rfPhyPktFmt   = PKT_FMT_ZIGBEE;
    //============config RF Frequency Offset
    g_rfPhyFreqOffSet = RF_PHY_FREQ_FOFF_20KHZ;

    XTAL16M_CAP_SETTING(0x09);
    XTAL16M_CURRENT_SETTING(0x01);

    hal_rom_boot_init();

    NVIC_SetPriority((IRQn_Type)BB_IRQn,    IRQ_PRIO_REALTIME);
    NVIC_SetPriority((IRQn_Type)TIM1_IRQn,  IRQ_PRIO_HIGH);     //ll_EVT
    NVIC_SetPriority((IRQn_Type)TIM2_IRQn,  IRQ_PRIO_HIGH);     //OSAL_TICK
    NVIC_SetPriority((IRQn_Type)TIM4_IRQn,  IRQ_PRIO_HIGH);     //LL_EXA_ADV

    //hal_pwrmgr_register(MOD_USR0, NULL, rf_wakeup_handler);


}
comms_error_t radio_stop(comms_layer_iface_t* interface, comms_status_change_f* cb, void* user)
{
	osThreadFlagsSet(m_config.threadid, RDFLG_RADIO_STOP);
	return COMMS_SUCCESS;
}

comms_error_t radio_start(comms_layer_iface_t* interface, comms_status_change_f* cb, void* user)
{
	osThreadFlagsSet(m_config.threadid, RDFLG_RADIO_START);
	return COMMS_SUCCESS;
}

static void radio_send_timeout_cb (void * argument)
{
    osThreadFlagsSet(m_config.threadid, RDFLG_RADIO_SEND_TIMEOUT);
}

static void radio_resend_timeout_cb(void * argument)
{
    osThreadFlagsSet(m_config.threadid, RDFLG_RADIO_RESEND);
}

static void radio_send_message (comms_msg_t * msg)
{
    __align(4) static uint8_t buffer[160] = {0};

    if (NULL == msg)
    {
        sys_panic("snull");
    }

    comms_layer_t* iface = (comms_layer_t *)&m_radio_iface;
    //RAIL_Status_t rslt;
    uint16_t count, total;
    uint16_t src, dst;
    uint8_t amid;
    //RAIL_CsmaConfig_t csmaConf = {0, 0, 1, -75, 320, 128, 0};

    count = comms_get_payload_length(iface, msg);
    src = comms_am_get_source(iface, msg);
    if (src == 0)
    {
        src = m_config.nodeaddr;
    }
    dst = comms_am_get_destination(iface, msg);
    if (dst == 0)
    {
        warn1("dest not set");
    }
    amid = comms_get_packet_type(iface, msg);
    // is ack and not broadcast
    if (comms_is_ack_required(iface, msg) && (dst != 0xFFFF))
    {
        radio_tx_wait_ack = true;
        buffer[1] = 0x61;
    }
    else
    {
        radio_tx_wait_ack = false;
        buffer[1] = 0x41;
    }

    buffer[2] = 0x88;
    buffer[3] = m_radio_tx_num;
    buffer[4] = ((m_config.pan >> 0) & (0xFF));
    buffer[5] = ((m_config.pan >> 8) & (0xFF));
    buffer[6] = ((dst >> 0) & 0xFF);
    buffer[7] = ((dst >> 8) & 0xFF);
    buffer[8] = ((src >> 0) & 0xFF);
    buffer[9] = ((src >> 8) & 0xFF);
    buffer[10] = 0x3F;
    // AMID handled below
    memcpy(&buffer[12], comms_get_payload(iface, msg, count), count);

		zb_hw_set_stx();
    ll_hw_rst_tfifo();
		
    // Pick correct AMID, add timestamp footer when needed
    if (comms_event_time_valid(iface, msg))
    {
        uint32_t evt_time, diff;
        //debug1("evt time valid");
        buffer[11] = 0x3d; // 3D is used by TinyOS AM for timesync messages

        evt_time = comms_get_event_time_us(iface, msg);
        diff = evt_time - (radio_timestamp()+1000); // It will take at least 448us to get the packet going, round it up

			  //LOG("diff: %d\r\n", diff);
        buffer[12+count] = amid; // Actual AMID is carried after payload
        buffer[13+count] = diff>>24;
        buffer[14+count] = diff>>16;
        buffer[15+count] = diff>>8;
        buffer[16+count] = diff;
        count += 5;
    }
    else
    {
        //debug1("evt time NOT valid");
        buffer[11] = amid;
    }

    buffer[0] = 11 + count + 2; // hdr, data, crc
    total = 1 + 11 + count + 2; // lenb, hdr, data, crc

    //RAIL_WriteTxFifo(m_rail_handle, buffer, count, true);
    //RAIL_SetTxFifo(m_rail_handle, buffer, total, sizeof(buffer));

	  uint8_t crcCode[2] = {0};
		uint8_t seed[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

		//zigbee_crc16_gen(&buffer[12], count, seed, crcCode);

    buffer[1 + 11 + count + 1] = crcCode[0];
    buffer[1 + 11 + count + 2] = crcCode[1];

		ll_hw_write_tfifo(&buffer[0], total);
		m_radio_send_timestamp = radio_timestamp();
		ll_hw_go();

		osTimerStart(m_send_timeout_timer, RADIO_MAX_SEND_TIME_MS);

    // if ack is required in FCF
		/*
    if (radio_tx_wait_ack)
    {
        //rslt = RAIL_StartCcaCsmaTx(m_rail_handle, m_radio_channel_current, RAIL_TX_OPTION_WAIT_FOR_ACK, &csmaConf, NULL);
    }
    else
    {
        //rslt = RAIL_StartCcaCsmaTx(m_rail_handle, m_radio_channel_current, 0, &csmaConf, NULL);

    }

    debug1("snd %04"PRIX16"->%04"PRIX16"[%02"PRIX8"](%"PRIx8":%"PRIu8")=%d %p l:%d",
           src, dst, amid, m_radio_tx_num, m_csma_retries, rslt, msg, total);
		*/


		/*
    if (rslt == RAIL_STATUS_NO_ERROR)
    {
        osTimerStart(m_send_timeout_timer, RADIO_MAX_SEND_TIME_MS);
    }

    else
    {
        //RAIL_Idle(m_rail_handle, RAIL_IDLE_FORCE_SHUTDOWN, 1);
        //RAIL_StartRx(m_rail_handle, m_radio_channel_current, NULL);
        osThreadFlagsSet(m_config.threadid, RDFLG_RADIO_SEND_FAIL);
    }
		*/
}


static void signal_send_done (comms_error_t err)
{
    comms_send_done_f * send_done = NULL;
    comms_msg_t * msgp;
    void * user;
    uint32_t qtime;

    osTimerStop(m_send_timeout_timer);

    //assert(NULL != radio_msg_sending);

    user = radio_msg_sending->user;
    msgp = radio_msg_sending->msg;
    send_done = radio_msg_sending->send_done;
    qtime = radio_timestamp() - radio_msg_sending->timestamp_queued;

    radio_msg_sending->next = (radio_queue_element_t*)radio_msg_queue_free;
    radio_msg_queue_free = radio_msg_sending;
    radio_msg_sending = NULL;

    if (err == COMMS_SUCCESS)
    {
			comms_set_timestamp_us((comms_layer_t *)&m_radio_iface, msgp, radio_timestamp()); // TODO: Crashes
        _comms_set_ack_received((comms_layer_t *)&m_radio_iface, msgp);
    }

    if (qtime > 20)
    {
        //warn3("slow tx %"PRIu32, qtime);
    }

		/*
    logger(err==COMMS_SUCCESS?LOG_INFO3:LOG_WARN3,
          "snt %p e:%d t:(%"PRIu32")(%"PRIu32")",
           msgp, err,
           qtime,
           m_rail_sent_timestamp - m_rail_send_timestamp);
		*/

    //assert(NULL != send_done);
    send_done((comms_layer_t *)&m_radio_iface, msgp, err, user);
}


static void handle_radio_tx (uint32_t flags)
{

    // If sending, see if it has completed
    if (NULL != radio_msg_sending)
    {
        // Sending has completed -----------------------------------------------
        if (flags & RDFLG_RAIL_SEND_DONE)
        {
            //update_tx_stats(radio_msg_sending->msg); TODO

            if (radio_tx_wait_ack) // Alternatively we should get rx_ack_timeout
            {
                debug1("ackd");
                comms_ack_received((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg);
            }
            signal_send_done(COMMS_SUCCESS);
        }
        // Ack was not received ------------------------------------------------
        else if (flags & RDFLG_RAIL_RXACK_TIMEOUT)
        {
            bool resend = false;

            osTimerStop(m_send_timeout_timer);

            //update_tx_stats(radio_msg_sending->msg);

            if (comms_get_retries_used((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg)
             < comms_get_retries((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg))
            {
                resend = true;
            }

            /*logger(resend?LOG_DEBUG1:LOG_WARN3, "rx ackTimeout (%"PRIu8"/%"PRIu8")",
                   comms_get_retries_used((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg),
                   comms_get_retries((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg));
						*/
            if (resend)
            {
                osTimerStart(m_resend_timer,
                             comms_get_timeout((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg));
            }
            else
            {
                signal_send_done(COMMS_ENOACK);
            }
        }
        // CSMA has failed to transmit the message -----------------------------
        else if (flags & RDFLG_RAIL_SEND_BUSY)
        {
            bool resend = false;

            osTimerStop(m_send_timeout_timer);


            if (resend)
            {
                zb_hw_stop();
                radio_send_message(radio_msg_sending->msg);
            }
            else
            {
                if (comms_get_retries_used((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg)
                 < comms_get_retries((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg))
                {
                    osTimerStart(m_resend_timer, comms_get_timeout((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg));
                }
                else
                {
                    signal_send_done(COMMS_EBUSY);
                }
            }
        }
        // Sending has failed in some generic way ------------------------------
        else if (flags & RDFLG_RADIO_SEND_FAIL)
        {
            signal_send_done(COMMS_FAIL);
        }
        // Sending has not completed in a reasonable amount of time ------------
        else if (flags & RDFLG_RADIO_SEND_TIMEOUT)
        {
            // Check that an actual timeout has happened and this is not race
            uint32_t passed = radio_timestamp() - m_radio_send_timestamp;
            if (passed >= RADIO_MAX_SEND_TIME_MS)
            {
                err1("TIMEOUT");
                signal_send_done(COMMS_ETIMEOUT);

                // Presumably something is wrong with the radio
                osThreadFlagsSet(m_config.threadid, RDFLG_RADIO_RESTART);
            }
            else // Perhaps triggered because timer from previous send was not stopped in time
            {
                //warn1("timeout %"PRIu32, passed);
                osTimerStart(m_send_timeout_timer, RADIO_MAX_SEND_TIME_MS - passed);
            }
        }
        // PacketLink resend functionality -------------------------------------
        else if (flags & RDFLG_RADIO_RESEND)
        {
            //radio_resend();
        }
    }
}


static void handle_radio_rx()
{
		uint8_t buffer[140];
    // RX processing -----------------------------------------------------------
    if (osOK == osMessageQueueGet(m_config.recvQueue, &buffer, NULL, 0))
    {
			uint8_t len = buffer[0];
			uint32_t rts = (buffer[len] << 24) | (buffer[len+1] << 16) | (buffer[len+2] << 8) | buffer[len+3];
			
			if ((buffer[2] == 0x88)&&(buffer[5] == 0x00) && (buffer[10] == 0x3F))
			{
				comms_msg_t msg;
				am_id_t amid;
				void* payload;
				uint8_t plen = len -13;
				uint8_t lqi = 0xFF;
				uint16_t source = ((uint16_t)buffer[8] << 0) | ((uint16_t)buffer[9] << 8);
				uint32_t currTime = radio_timestamp();
				uint32_t timestamp = currTime - (currTime - rts);

				comms_init_message((comms_layer_t *)&m_radio_iface, &msg);
				
				if (buffer[11] == 0x3D)
				{
						int32_t diff = (buffer[len - 5] << 24) | (buffer[len - 4] << 16) | (buffer[len- 3] << 8) | (buffer[len- 2]);
	
						if ((len < 17)
							||(len > 200))
						{
								sys_panic("packet");
						}
						amid = buffer[(len-6)];
						plen = len - 18;

						comms_set_event_time_us((comms_layer_t *)&m_radio_iface, &msg, (uint32_t)(diff + timestamp));
						
				}
				else
				{
						amid = buffer[11];
						//plen = packetInfo.packetBytes - 12;
				}
				
				payload = comms_get_payload((comms_layer_t *)&m_radio_iface, &msg, plen);

				if (NULL != payload)
				{
						uint16_t dest = ((uint16_t)buffer[6] << 0) | ((uint16_t)buffer[7] << 8);

						comms_set_packet_type((comms_layer_t *)&m_radio_iface, &msg, amid);
						comms_set_payload_length((comms_layer_t *)&m_radio_iface, &msg, plen);
						memcpy(payload, (const void *)&buffer[12], plen);
				
						comms_set_timestamp_us((comms_layer_t *)&m_radio_iface, &msg, rts);
				
						_comms_set_rssi((comms_layer_t *)&m_radio_iface, &msg, -60);
						
						lqi = lqi + (-60 + 93)*50;
						
						_comms_set_lqi((comms_layer_t *)&m_radio_iface, &msg, lqi);
						comms_am_set_destination((comms_layer_t *)&m_radio_iface, &msg, dest);
						comms_am_set_source((comms_layer_t *)&m_radio_iface, &msg, source);
						
						//LOG("rx: %04X->%04X[%02X]\r\n", comms_am_get_source((comms_layer_t *)&m_radio_iface, &msg), comms_am_get_destination((comms_layer_t *)&m_radio_iface, &msg), comms_get_packet_type((comms_layer_t *)&m_radio_iface, &msg));

						comms_deliver((comms_layer_t *)&m_radio_iface, &msg);
				}
				
				
			}
/*        RAIL_RxPacketInfo_t packetInfo = {0};
        RAIL_RxPacketHandle_t packetHandle = RAIL_GetRxPacketInfo(m_rail_handle, rxh, &packetInfo);
        if (packetHandle != RAIL_RX_PACKET_HANDLE_INVALID)
        {
            RAIL_RxPacketDetails_t packetDetails = {0};
            RAIL_Status_t rx_status = RAIL_GetRxPacketDetailsAlt(m_rail_handle, packetHandle, &packetDetails);
            if (rx_status == RAIL_STATUS_NO_ERROR)
            {
                uint8_t buffer[256] = {0};
                RAIL_RxPacketDetails_t timeDetails = packetDetails;
                bool rts_valid = timeDetails.timeReceived.timePosition != RAIL_PACKET_TIME_INVALID;
                uint32_t rts = 0; // timeReceived.packetTime

                assertPacketInfo(&packetInfo); // Sanity checks

                if (rts_valid)
                {
                    // Account for CRC ... unless someone somewhere configures RAIL_RX_OPTION_STORE_CRC?
                    timeDetails.timeReceived.totalPacketBytes = packetInfo.packetBytes + 2; // + CRC_BYTES;
                    // Want the earliest timestamp possible
                    rx_status = RAIL_GetRxTimePreambleStartAlt(m_rail_handle, &timeDetails);
                    if (rx_status == RAIL_STATUS_NO_ERROR)
                    {
                        rts = timeDetails.timeReceived.packetTime;
                    }
                    else
                    {
                        rts_valid = false;
                    }
                }

                if (rts_valid == false)
                {
                    warn1("rts %d %d", packetDetails.timeReceived.timePosition, timeDetails.timeReceived.timePosition);
                }

                RAIL_CopyRxPacket(buffer, &packetInfo);

                RAIL_Status_t rst = RAIL_ReleaseRxPacket(m_rail_handle, packetHandle);
                if (rst != RAIL_STATUS_NO_ERROR)
                {
                    warnb1("rst", &rst, sizeof(RAIL_Status_t));
                    sys_panic("release");
                }

								uint8_t buffer[256] = {0};
                uint16_t currTime = (uint16_t)(radio_timestamp() >> 10);
								
                uint16_t source = ((uint16_t)buffer[8] << 0) | ((uint16_t)buffer[9] << 8);


                if ((!radio_seqNum_save(source, buffer[3], currTime)) && (packetInfo.packetBytes >= 12))
                {
                    warn3("same seqNum:%02"PRIX8, buffer[3]);
                }
                else if ((packetInfo.packetBytes >= 12)
                       &&(buffer[2] == 0x88)&&(buffer[5] == 0x00) && (buffer[10] == 0x3F))
                {
                    comms_msg_t msg;
                    am_id_t amid;
                    void* payload;
                    uint8_t plen;
                    uint8_t lqi = 0xFF;
                    uint32_t timestamp = radio_timestamp() - (RAIL_GetTime() - rts)/1000;

                    comms_init_message((comms_layer_t *)&m_radio_iface, &msg);
                    if (buffer[11] == 0x3D)
                    {
                        int32_t diff = (buffer[packetInfo.packetBytes - 4] << 24) |
                                   (buffer[packetInfo.packetBytes - 3] << 16) |
                                   (buffer[packetInfo.packetBytes - 2] << 8) |
                                   (buffer[packetInfo.packetBytes - 1]);

                        if ((packetInfo.packetBytes < 17)
                          ||(packetInfo.packetBytes > 200))
                        {
                            sys_panic("packet");
                        }
                        amid = buffer[(packetInfo.packetBytes-5)];
                        plen = packetInfo.packetBytes - 17;
                        if (rts_valid)
                        {
                            comms_set_event_time((comms_layer_t *)&m_radio_iface, &msg, (uint32_t)(diff + timestamp));
                        }
                    }
                    else
                    {
                        amid = buffer[11];
                        plen = packetInfo.packetBytes - 12;
                    }

                    payload = comms_get_payload((comms_layer_t *)&m_radio_iface, &msg, plen);

                    if (NULL != payload)
                    {
                        uint16_t dest = ((uint16_t)buffer[6] << 0) | ((uint16_t)buffer[7] << 8);

                        comms_set_packet_type((comms_layer_t *)&m_radio_iface, &msg, amid);
                        comms_set_payload_length((comms_layer_t *)&m_radio_iface, &msg, plen);
                        memcpy(payload, (const void *)&buffer[12], plen);

                        if (rts_valid)
                        {
                            comms_set_timestamp((comms_layer_t *)&m_radio_iface, &msg, timestamp);
                        }
                        _comms_set_rssi((comms_layer_t *)&m_radio_iface, &msg, packetDetails.rssi);
                        if (packetDetails.rssi < -96)
                        {
                            lqi = 0;
                        }
                        else if (packetDetails.rssi < -93) // RFR2-like LQI simulation
                        {
                            lqi = lqi + (packetDetails.rssi+93)*50;
                        }
                        _comms_set_lqi((comms_layer_t *)&m_radio_iface, &msg, lqi);
                        comms_am_set_destination((comms_layer_t *)&m_radio_iface, &msg, dest);
                        comms_am_set_source((comms_layer_t *)&m_radio_iface, &msg, source);

                        debugb1("rx %04"PRIX16"->%04"PRIX16"[%02"PRIX8"] %"PRIu32" r:%"PRIi8" l:%"PRIu8" %"PRIu8":",
                                &(buffer[12]), 8,
                                source, dest, amid,
                                rts,
                                packetDetails.rssi, lqi, plen);

                        debug2("Rx[%02"PRIX8"] %04"PRIX16"->%04"PRIX16" rssi:%"PRIi8"",
                                buffer[12],
                                source, dest,
                                packetDetails.rssi);


                        comms_deliver((comms_layer_t *)&m_radio_iface, &msg);
                    }
                    else LOG("rx bad pl ");
                }
                else LOG("rx bad ");
            }
            else LOG("rxd");
        }
        else LOG("rxi");


        // There might be more packets in the queue, but don't let RX swamp the
        // radio - defer it to the next run through the loop
        osThreadFlagsSet(m_config.threadid, RDFLG_RAIL_RX_MORE);
				*/
    }
}


static void radio_send_next()
{
    comms_msg_t * msg = NULL;
    while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever));
    if (NULL != radio_msg_queue_head)
    {
        radio_msg_sending = radio_msg_queue_head;
        radio_msg_queue_head = radio_msg_queue_head->next;
        m_radio_tx_num++;

        msg = radio_msg_sending->msg;
    }
    osMutexRelease(m_radio_mutex);

    if (NULL != msg)
    {
        zb_hw_stop();
        radio_send_message(msg);
    }
}

static void handle_radio_events ()
{
    // RX busy handling -----------------------------------------------------
    if (rx_busy || rx_overflow)
    {
			/*
        uint8_t rxb __attribute__((unused));
        uint8_t rxo __attribute__((unused));
        CORE_irqState_t irqState = CORE_EnterCritical();
        rxb = rx_busy;
        rxo = rx_overflow;
        rx_busy = 0;
        rx_overflow = 0;
        CORE_ExitCritical(irqState);
			
        warn1("rx b:%"PRIu8" o:%"PRIu8, rxb, rxo);
			*/
    }

    // RX failure handling -----------------------------------------------------
    if ((rx_abort > 100) || rx_fail)
    {
			/*
        uint8_t rxa __attribute__((unused));
        uint8_t rxf __attribute__((unused));
        CORE_irqState_t irqState = CORE_EnterCritical();
        rxa = rx_abort;
        rxf = rx_fail;
        rx_abort = 0;
        rx_fail = 0;
        CORE_ExitCritical(irqState);
        warn1("rx a:%"PRIu8" f:%"PRIu8, rxa, rxf);
			*/
    }

    // RX frame error handling -----------------------------------------------------
    if (rx_frame_error)
    {
			/*
        uint8_t rxfe __attribute__((unused));
        CORE_irqState_t irqState = CORE_EnterCritical();
        rxfe = rx_frame_error;
        rx_frame_error = 0;
        CORE_ExitCritical(irqState);
        warn1("rx fe:%"PRIu8, rxfe);
			*/
    }

    // TX ack sent -----------------------------------------------------------------
    if (tx_ack_sent)
    {
			/*
        uint8_t tas __attribute__((unused));
        CORE_irqState_t irqState = CORE_EnterCritical();
        tas = tx_ack_sent;
        tx_ack_sent = 0;
        CORE_ExitCritical(irqState);
        info4("tx_ack_sent:%"PRIu8, tas);
			*/
    }
}


static void radio_task(void *arg)
{
	while(1)
	{

		//bool running = false;

		uint32_t flags = osThreadFlagsWait(RDFLGS_ALL, osFlagsWaitAny, osWaitForever);
		uint32_t state;

		 while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever))
        state = m_state;
        osMutexRelease(m_radio_mutex);

		 /*
        if (ST_STARTING == state)
        {
            //start_radio_now();
            running = true;
        }
		*/
        // If an exception has occurred and RAIL is broken ---------------------
        if (flags & RDFLG_RADIO_RESTART)
        {
            warn1("restart");
						zb_hw_stop();

					/*
            m_rail_handle = radio_rail_init();
            if (m_rail_handle == NULL)
            {
                sys_panic("rail");
            }
					*/

            // If sending, cancel and notify user
            if (NULL != radio_msg_sending)
            {
                flags |= RDFLG_RADIO_SEND_FAIL;
            }
        }

        // Handle TX activities
        handle_radio_tx(flags);
				
				/*
				zb_hw_stop();
				zb_hw_set_srx(10000);
				ll_hw_rst_rfifo();
				zb_hw_go();
*/
        // Check RX queue and process any messages there
        handle_radio_rx(); //TODO: uncomment

        // Handle "other" events
        //handle_radio_events();

        if (radio_msg_sending == NULL)
        {
            if (ST_STOPPING == state)
            {
                zb_hw_stop(); // Will return queued messages
                //running = false;
            }
            else
            {
                radio_send_next(); // Won't do anything if nothing queued
            }
        }

        if (flags & RDFLG_RADIO_DEINIT)
        {
            if (state == ST_OFF)
            {
                debug1("deinit");
                osThreadExit();
            }
        }
	}
}
radio_config_t* init_radio(uint16_t nodeaddr, uint8_t channel, uint8_t pan)
{

		if(channel < 10 || channel > 26)
		{
			 return NULL;
		}


		m_config.channel = channel;

		hal_rfphy_init();

		zb_set_channel(channel);

		zb_hw_set_srx(0);

		m_config.nodeaddr = nodeaddr;
		m_config.pan = pan;

		int res = comms_am_create((comms_layer_t *)&m_radio_iface, nodeaddr, radio_send, radio_start, radio_stop);

		if(COMMS_SUCCESS != res)
		{
			return NULL;
		}

		m_config.radio = (comms_layer_t *)&m_radio_iface;

		JUMP_FUNCTION(V4_IRQ_HANDLER)  =   (uint32_t)&ZBRFPHY_IRQHandler;
		m_config.recvQueue = osMessageQueueNew(10,140,NULL);


		if(NULL == m_config.recvQueue)
		{
				return NULL;
		}

		const osThreadAttr_t main_thread_attr = {.name = "radio"};
    m_config.threadid = osThreadNew(radio_task, NULL, &main_thread_attr);

		if(NULL == m_config.threadid)
		{
				return NULL;
		}


		radio_msg_sending = NULL;
    radio_msg_queue_head = NULL;
    radio_msg_queue_free = &radio_msg_queue_memory[0];
    radio_msg_queue_free->next = NULL;
    for(uint8_t i=1;i<sizeof(radio_msg_queue_memory)/sizeof(radio_queue_element_t);i++)
    {
        radio_msg_queue_memory[i].next = (radio_queue_element_t*)radio_msg_queue_free;
        radio_msg_queue_free = &radio_msg_queue_memory[i];
    }


		m_radio_mutex = osMutexNew(NULL);
    m_send_timeout_timer = osTimerNew(&radio_send_timeout_cb, osTimerOnce, NULL, NULL);
    m_resend_timer = osTimerNew(&radio_resend_timeout_cb, osTimerOnce, NULL, NULL);

		m_state = ST_RUNNING;

		memset(&m_rxBuf[0],0x0,127);

		zb_hw_go();

	 return &m_config;
}

void radio_deinit (comms_layer_t * iface)
{
    if (iface == (comms_layer_t*)&m_radio_iface)
    {
        osThreadFlagsSet(m_config.threadid, RDFLG_RADIO_DEINIT);
        // osThreadJoin not implented in all CMSIS adapters (FreeRTOS)
        while (osThreadTerminated != osThreadGetState(m_config.threadid))
        {
            osThreadYield(); // Wait for termination ...
        }
    }
}