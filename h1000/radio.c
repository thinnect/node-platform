/**
 * Radio driver for h1002.
 *
 * Copyright Thinnect Inc. 2021
 * @license MIT
*/
#include "loglevels.h"
#define __MODUUL__ "radio"
#define __LOG_LEVEL__ ( LOG_LEVEL_radio & 0xFFFF )
#include "log.h"


#include "radio.h"
#include "bus_dev.h"
#include "sys_panic.h"
#include "assert.h"


//#define LOG_TX_TIMESTAMPS 1
//#define LOG_RX_TIMESTAMPS 1
#define USE_ACK 1

#define RADIO_MAX_SEND_TIME_MS 50UL
#define RADIO_WAIT_FOR_ACK_MS 10UL // 864us
#define RADIO_WAIT_FOR_ACK_SENT_MS 5
#define RADIO_WAIT_HW_STOP_CNT 100
#define RADIO_BROADCAST_ADDR 0xFFFF

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
#define RDFLG_RADIO_ACK           (1 << 21)
#define RDFLG_RADIO_STRT_ACK_TIM  (1 << 22)
#define RDFLG_ACK_START           (1 << 23)
#define RDFLG_ACK_SENT_TIMEOUT    (1 << 24)
#define RDFLG_CRC_ERROR           (1 << 26)
#define RDFLG_QUEUE_ERROR         (1 << 27)

#define RDFLGS_ALL                (0x7FFFFFFF)

#define LL_HW_MODE_STX             0x00
#define LL_HW_MODE_SRX             0x01
#define LL_HW_MODE_TRX             0x10
#define LL_HW_MODE_RTX             0x12
#define LL_HW_MODE_TRLP            0x20
#define LL_HW_MODE_RTLP            0x30

// LL engine settle time
#define LL_HW_BB_DELAY_VAL         32
#define LL_HW_AFE_DELAY_VAL        8
#define LL_HW_PLL_DELAY_VAL        52
#define SCAN_RSP_DELAY_VAL         32
#define MAX_RX_TIMEOUT             0

// queue -----------------------------------------------------------------------
typedef struct radio_queue_element radio_queue_element_t;
struct radio_queue_element
{
    comms_msg_t* msg;
    comms_send_done_f* send_done;
    void* user;
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

static comms_error_t radio_start(comms_layer_iface_t* interface, comms_status_change_f* cb, void* user);
static comms_error_t radio_stop(comms_layer_iface_t* interface, comms_status_change_f* cb, void* user);
static comms_error_t radio_send(comms_layer_iface_t* interface, comms_msg_t* msg, comms_send_done_f* cb, void* user);

typedef enum RadioState
{
    ST_UNINITIALIZED,
    ST_OFF,
    ST_STARTING,
    ST_RUNNING,
    ST_STOPPING
} RadioState_t;

extern void hal_rom_boot_init(void);
extern volatile uint32 llWaitingIrq;
extern uint32_t ll_hw_get_tr_mode(void);


//debug stuff
extern void dbg_printf(const char *format, ...);
volatile int g_rf_tx_line;
volatile int g_rf_irq_line;
volatile int g_rf_irq_flag;
volatile int g_rf_irq_plen;
volatile int g_rf_irq_count;
volatile int g_rf_stp_line;
volatile int g_rf_stp_cnt;




static uint8_t m_rxBuf[127] = {0};
static uint16_t volatile m_irq_flag = 0;
static uint32_t m_foot[2] = {0};
static comms_layer_am_t m_radio_iface;
static volatile uint8_t rx_busy;
static volatile uint8_t rx_overflow;
static volatile uint8_t rx_frame_error;
static volatile uint8_t rx_abort;
static volatile uint8_t rx_fail;
static volatile uint8_t tx_ack_sent;

static uint32_t radio_timestamp ();
static volatile uint32_t m_hw_stop_start;
static volatile uint32_t m_hw_stop_end;

static volatile bool m_hw_stopping;

enum
{
    START_RADIO_SEND,
    RADIO_SEND_MSG,
    RADIO_SEND_MSG_PCKT_DONE,
    START_CHECK_ETHER,
    STOP_CHECK_ETHER,
    RF_TX,
    RF_TX_DONE,
    IRQ_SEND_DONE,
    SIGNAL_SEND_DONE,
    TX_LAST_TIMESTAMP
};
static volatile uint32_t tx_timestamps[TX_LAST_TIMESTAMP] = {0};

enum
{
    RX_IRQ_START,
    RX_IRQ_FINISH,
    SEND_ACK,
    SEND_DONE_ACK,
    RX_PRC_START,
    RX_PRC_END,
    ACK_DELAY,
    RX_LAST_TIMESTAMP
};
static volatile uint32_t rx_timestamps[RX_LAST_TIMESTAMP] = {0};
static osMutexId_t m_radio_mutex;

static radio_config_t m_config = {0};
static uint8_t m_radio_tx_num;
static bool radio_tx_wait_ack;
static RadioState_t m_state = ST_UNINITIALIZED;

static osTimerId_t m_send_timeout_timer;
static osTimerId_t m_resend_timer;
static osTimerId_t m_ack_timer;
static osTimerId_t m_ack_timeout_timer;

static uint32_t m_radio_send_timestamp;

static uint8_t m_csma_retries;

static comms_status_change_f* m_state_change_cb;
static void* m_state_change_user;

static uint8_t m_radio_channel_configured;
static uint8_t m_radio_channel_current;

static uint32_t m_stop_timestamp;

static bool transfer_pending = false;
static bool sending_ack = false;
static uint8_t ack_seq = 0;

static uint32_t max_fine_time = 0;
static uint32_t max_carr_cnt;

// Packets actually transmitted (energy for TX spent)
static uint32_t m_transmitted_packets;

// Bytes actually transmitted (energy for TX spent)
static uint32_t m_transmitted_bytes;

static void update_tx_stats(comms_msg_t * msg)
{
    m_transmitted_packets++; // Packet was actually sent out
    m_transmitted_bytes += (14 // 12 bytes header + 2 bytes CRC
        + comms_get_payload_length((comms_layer_t *)&m_radio_iface, msg)
        + (comms_event_time_valid((comms_layer_t *)&m_radio_iface, msg) ? 5 : 0)
        + 2);
}

static void zb_hw_go (void)
{
    *(volatile uint32_t*)(LL_HW_BASE + 0x14) = LL_HW_IRQ_MASK;    //clr  irq status
    *(volatile uint32_t*)(LL_HW_BASE + 0x0C) = 0x0001;            //mask irq :only use mode done
    *(volatile uint32_t*)(LL_HW_BASE + 0x00) = 0x0001;            //trig

    uint8_t rfChnIdx = PHY_REG_RD(0x400300B4) & 0xFF;

    if (rfChnIdx < 2)
    {
        rfChnIdx = 2;
    }
    else if (rfChnIdx > 80)
    {
        rfChnIdx = 80;
    }

    rf_tpCal_cfg(rfChnIdx);
    // subWriteReg(0x40030094,7,0,g_rfPhyTpCalCapArry[(rfChnIdx-2)>>1]);

#if (DBG_BUILD_LL_TIMING)
    hal_gpio_write(DBG_PIN_LL_HW_TRIG, 1);
    hal_gpio_write(DBG_PIN_LL_HW_TRIG, 0);
#endif
}

static uint32_t fine_time_passed (uint32_t ts)
{
    uint32_t now = read_current_fine_time();
    if (ts > now) // The fine-time counter has wrapped its 22-bit TOP
    {
        now += 4194304; // Add 22-bits worth of time
    }
    return now - ts;
}


static void blinkdeath (void) // TODO: remove this
{
    for (;;)
    {
        gpio_write(P25,1);
        gpio_write(P24,1);
        gpio_write(P23,1);
        uint32_t start = read_current_fine_time();
        while (fine_time_passed(start) < 100000);
        gpio_write(P25,0);
        gpio_write(P24,0);
        gpio_write(P23,0);
        start = read_current_fine_time();
        while (fine_time_passed(start) < 100000);
    }
}

static bool zb_hw_stop (void)
{
    //int32_t lock = osKernelLock();
    uint32_t stop_start_fine = read_current_fine_time();

    HAL_ENTER_CRITICAL_SECTION();
    if (m_hw_stopping)
    {
        dbg_printf("m_stopping_hw");
        while(1);
    }
    m_hw_stopping = true;
    g_rf_irq_count = 0;
    g_rf_stp_cnt = 0;
		uint32_t mode = ll_hw_get_tr_mode();
    HAL_EXIT_CRITICAL_SECTION();

    uint32_t stop_force_fine = read_current_fine_time();
    ll_hw_set_rx_timeout(5); // will trigger ll_hw_irq=RTO

    while (true == m_hw_stopping)
    {
        if (fine_time_passed(stop_force_fine) > 12000) // ~12ms
        {
            stop_force_fine = read_current_fine_time();
            ll_hw_set_rx_timeout(5); // will trigger ll_hw_irq=RTO
            g_rf_stp_cnt++;
        }

        //WaitRTCCount(1);

        if (fine_time_passed(stop_start_fine) > 1000000) // 1 second
        {
            HAL_ENTER_CRITICAL_SECTION();
            dbg_printf("TIMEOUT %u %u %u\n", (unsigned int)(read_current_fine_time()),
                (unsigned int)stop_start_fine, (unsigned int)(read_current_fine_time() - stop_start_fine));
						dbg_printf("L T:%d I:%d C=%d F=%X\r\n", g_rf_tx_line, g_rf_irq_line, g_rf_irq_count, g_rf_irq_flag);
            dbg_printf("L S:%d C=%d\r\n", g_rf_stp_line, g_rf_stp_cnt);
            dbg_printf("mode: %d\n", (int)(mode));
            blinkdeath();
            while(1);
        }
    };

    //osKernelRestoreLock(lock);

    int stpt = fine_time_passed(stop_start_fine);
   // logger(stpt > 1000 ? LOG_WARN1: LOG_DEBUG2, "stpt %d (%d)", stpt, g_rf_stp_cnt);

    return true;
}


static void zb_hw_timing (void)
{
    //restore the ll register
    ll_hw_set_tx_rx_release	(10, 1);
    ll_hw_set_rx_tx_interval(98);		// T_IFS = 192+2us for ZB 98
    ll_hw_set_tx_rx_interval(108);		// T_IFS = 192-6us for ZB 108
    ll_hw_set_trx_settle(LL_HW_BB_DELAY_VAL, LL_HW_AFE_DELAY_VAL, LL_HW_PLL_DELAY_VAL);    // TxBB, RxAFE, PLL
}

static void zb_hw_set_srx (uint32_t rxTimeOutUs)
{
    ll_hw_set_rx_timeout(rxTimeOutUs);
    ll_hw_set_srx();
    ll_hw_set_trx_settle(LL_HW_BB_DELAY_VAL, LL_HW_AFE_DELAY_VAL, LL_HW_PLL_DELAY_VAL);          //RxAFE,PLL
}

static void zb_hw_set_stx (void)
{
    ll_hw_set_stx();
    ll_hw_set_trx_settle(LL_HW_BB_DELAY_VAL, LL_HW_AFE_DELAY_VAL, LL_HW_PLL_DELAY_VAL);          //RxAFE,PLL
}


static void zb_hw_set_trx (uint32_t rxTimeOutUs)
{
    ll_hw_set_rx_timeout(rxTimeOutUs);
    ll_hw_set_trx();
    ll_hw_set_trx_settle(LL_HW_BB_DELAY_VAL, LL_HW_AFE_DELAY_VAL, LL_HW_PLL_DELAY_VAL);          //RxAFE,PLL
}

static void zb_set_channel (uint8_t chn)
{
    uint32_t rfChnIdx = (chn - 10) * 5;

    if(g_rfPhyFreqOffSet >= 0)
    {
        PHY_REG_WT(0x400300B4, (g_rfPhyFreqOffSet << 16) + (g_rfPhyFreqOffSet << 8) + rfChnIdx);
    }
    else 
    {
        PHY_REG_WT(0x400300B4, ((255 + g_rfPhyFreqOffSet) << 16) + ((255 + g_rfPhyFreqOffSet) << 8) + (rfChnIdx - 1) );
    }
    ll_hw_ign_rfifo(LL_HW_IGN_CRC);
}

static uint8_t  zbll_hw_read_rfifo_zb (uint8_t* rxPkt, uint16_t* pktLen, uint32_t* pktFoot0, uint32_t* pktFoot1)
{
    int rdPtr, wrPtr, rdDepth, blen, wlen;
    uint32_t* p_rxPkt=(uint32_t*)rxPkt;

    ll_hw_get_rfifo_info(&rdPtr,&wrPtr,&rdDepth);

    if (rdDepth > 0)
    {
        *p_rxPkt++ = *(volatile uint32_t*)(LL_HW_RFIFO);

        blen = rxPkt[0];           //get the byte length for header
        wlen = 1 + ((blen) >> 2);  //blen included the 2byte crc

        while (p_rxPkt < (uint32_t *)rxPkt + wlen)
        {
            *p_rxPkt++ = *(volatile uint32_t*)(LL_HW_RFIFO);
        }

        *pktFoot0 = *(volatile uint32_t*)(LL_HW_RFIFO);
        *pktFoot1 = *(volatile uint32_t*)(LL_HW_RFIFO);

        *pktLen = blen + 1;
        return wlen;
    }
    else
    {
        rxPkt[0]  = 0;
        *pktFoot0 = 0;
        *pktFoot1 = 0;
        *pktLen   = 0;
        return 0;
    }
}

static uint32_t radio_timestamp ()
{
    return osKernelGetTickCount();
}

uint8_t rf_getRssi (void)
{
    uint8_t rssi_cur = 0;
    uint16_t foff = 0;
    uint8_t carrSens = 0;
    rf_phy_get_pktFoot(&rssi_cur, &foff, &carrSens);
    return rssi_cur;
}

uint8_t rf_carriersense (void)
{
    uint8_t rssi_cur = 0;
    uint16_t foff = 0;
    uint8_t carrSens = 0;
    rf_phy_get_pktFoot(&rssi_cur, &foff, &carrSens);
    //debug1("Carrier sense : %d \r\n", carrSens);
    return carrSens;
}

void rf_setRxMode (uint16_t timeout)
{
    uint8_t cnt = 0;
	  uint32_t mode = ll_hw_get_tr_mode();
    
		if(m_hw_stopping) { return; }
		
		if(__get_IPSR() == 0U || __get_PRIMASK() == 0U)
		{
			    if (LL_HW_MODE_SRX == mode || LL_HW_MODE_TRX == mode)
					{
								return;
					} 
					else if (LL_HW_MODE_STX == mode)
					{
						// if in tx state, abort the tx first
						zb_hw_stop();
					}
		}
    zb_hw_set_srx(timeout);
    // reset Rx/Tx FIFO
    ll_hw_rst_rfifo();
    ll_hw_rst_tfifo();
    ll_hw_go();
}

phy_sts_t checkEther (void)
{
		return PHY_CCA_IDLE;
	
	
    uint32_t carr_peak = 150;
    uint32_t carr = 0;
    uint32_t carr_cnt = 0;

    tx_timestamps[START_CHECK_ETHER] = radio_timestamp();

#ifdef LOG_MODE
    uint8_t mode;
    // check current radio mode
    mode = ll_hw_get_tr_mode();
    debug1("m_cfg:%u mode:%u", m_config.mode, mode);
#endif

    // Set Rx mode when needed
    rf_setRxMode(MAX_RX_TIMEOUT);
        
    int32_t lock = osKernelLock();
    volatile uint32_t start_time = read_current_fine_time();
    volatile uint32_t cur_time;
    volatile uint32_t elapsed_time = 0;

    while(elapsed_time < 128)
    {
        carr += rf_carriersense();
        carr_cnt++;
        cur_time = read_current_fine_time();
        if (cur_time > start_time)
        {
            elapsed_time += cur_time - start_time;
            start_time = cur_time;
        }
        else if (cur_time < start_time)
        {
            elapsed_time += start_time - cur_time;
            start_time = cur_time;
        }
        else
        {
            // do nothing
        }
    }
    osKernelRestoreLock(lock);
    // for debugging, delete later!
    max_fine_time = elapsed_time;
    max_carr_cnt = carr_cnt;

    tx_timestamps[STOP_CHECK_ETHER] = radio_timestamp();
    
    carr_peak = carr / carr_cnt;
    if (carr_peak < m_config.cca_treshhold)
    {
        return PHY_CCA_IDLE;
    }
    else
    {
        return PHY_CCA_BUSY;
    }
}

void RFPHY_IRQHandler (void)
{
    uint8_t mode;
    uint32_t T2;
    uint32_t delay;
    uint8_t zbRssi = 0;
    uint16_t zbFoff = 0;
    uint8_t zbCarrSens = 0;

    data_pckt_t packet = {0};
    uint8_t buffer[140] = {0};
    uint32_t rts = radio_timestamp();

    uint32_t ISR_entry_time = read_current_fine_time();

    m_irq_flag = ll_hw_get_irq_status();
		
		g_rf_irq_flag = m_irq_flag;
    g_rf_irq_count++;
        

    if (!(m_irq_flag & LIRQ_MD))          // only process IRQ of MODE DONE
    {
        //debug1("Mode is not done");
        ll_hw_clr_irq();
        return;
    }
    HAL_ENTER_CRITICAL_SECTION();

    mode = ll_hw_get_tr_mode();

    // ===================   mode TRX process 1
    // Tx, option: No Ack Need
    //if ((mode == LL_HW_MODE_STX) && (m_irq_flag & LIRQ_TD))
    if (m_irq_flag & LIRQ_TD)
    {
#ifdef USE_ACK
        if (sending_ack)
        {
            sending_ack = false;
            rx_timestamps[SEND_DONE_ACK] = radio_timestamp();
            osThreadFlagsSet(m_config.threadid, RDFLG_RAIL_TXACK_SENT);
                    
        }        
        else if (!radio_tx_wait_ack)
        {
            tx_timestamps[IRQ_SEND_DONE] = radio_timestamp();
            osThreadFlagsSet(m_config.threadid, RDFLG_RAIL_SEND_DONE);
                    
        }
        else if (radio_tx_wait_ack)
        {
            osThreadFlagsSet(m_config.threadid, RDFLG_RADIO_STRT_ACK_TIM);
        }
#else
        // do not use ACK!
        tx_timestamps[IRQ_SEND_DONE] = radio_timestamp();
        osThreadFlagsSet(m_config.threadid, RDFLG_RAIL_SEND_DONE);
#endif
        // enable Rx again if Tx mode is not activated
        if (LL_HW_MODE_SRX == mode)
        {
            rf_setRxMode(MAX_RX_TIMEOUT);
        }
    }
        // Rx mode
    else if ((mode == LL_HW_MODE_SRX || mode == LL_HW_MODE_TRX))
    {
        uint8_t  packet_len = 0;
        uint16_t pktLen = 0;

        rf_phy_get_pktFoot(&zbRssi, &zbFoff, &zbCarrSens);
        // read packet
        if (m_irq_flag & LIRQ_COK)
        {
            memset((void*)&rx_timestamps[0], 0x00, sizeof(rx_timestamps));
            rx_timestamps[RX_IRQ_START] = radio_timestamp();
            // TODO: check packet_len value!
            packet_len = zbll_hw_read_rfifo_zb(&buffer[0], &pktLen, &m_foot[0], &m_foot[1]);
            rf_phy_get_pktFoot_fromPkt(m_foot[0], m_foot[1], &zbRssi, &zbFoff, &zbCarrSens);
                    

        }
        else if (m_irq_flag & LIRQ_CERR)
        {
            pktLen = 0;
            rf_setRxMode(MAX_RX_TIMEOUT);
            osThreadFlagsSet(m_config.threadid, RDFLG_CRC_ERROR);
        }

        //check Tx option for ack tx the payload and MAC address
        if (pktLen)
        {
            // TODO: get rid of copy!
            memcpy(&packet.buffer[0], &buffer[0], 140);
            // do the filtering
            mac_frame_t* frame = (mac_frame_t*)&packet.buffer[1];

            // if ( ((fcf1 & MAC_FCF_FRAME_TYPE) == 0x02) && (pktLen == 6)) {
            //     lmac_ackCb(MAC_FCF_FRAME_PENDING_BIT & fcf1, *(rxPkt->psdu+3), ISR_entry_time, zbRssi, zbCarrSens);
            //     fDrop = 1; // ACK packet
            //     break;
            // }
            
            // do not use ACK!
            uint16_t dest1 = ((uint16_t)packet.buffer[6] << 0) | ((uint16_t)packet.buffer[7] << 8);

#ifdef USE_ACK
            // ACK packet received, set send done flag
            if ((packet.buffer[0] == 0x05) && (packet.buffer[1] == 0x02) && (packet.buffer[3] == m_radio_tx_num))
            {
                if (radio_tx_wait_ack)
                {
                    osThreadFlagsSet(m_config.threadid, RDFLG_RAIL_SEND_DONE);
                }
            }

            // Handle ACK request with highest priority
            if ((frame->frame_control[0] & MAC_FCF_ACK_REQ_BIT) && (dest1 == m_config.nodeaddr))
            {
                // info1("DEST: %x NODE: %x",dest1, m_config.nodeaddr); 
                // uint8_t seed[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                //uint8_t crcCode[2] = {0xff, 0xff};
                uint8_t ackTxBuf[10];
                
                sending_ack = true;
                ack_seq = packet.buffer[3]; // ??? FOR SURE THIS GONNA BRING PROBLEMS LATER ON
                // calc and fill ack packet first
                ackTxBuf[0] = 0x05;
                ackTxBuf[1] = 0x02;
                ackTxBuf[2] = 0x00;
                ackTxBuf[3] = ack_seq; // SN number
                // TODO: Do we need this???
                // if (isDataPending(&srcAddr))
                // {
                //     ackTxBuf[1] |= 0x10;
                // }
                
                // send ack response
                zb_hw_set_stx();             // set LL HW as single Tx mode  
                // calculate the delay
                T2 = read_current_fine_time();

                delay = (T2 > ISR_entry_time) ? (T2 - ISR_entry_time) : (BASE_TIME_UNITS - ISR_entry_time + T2);
                rx_timestamps[ACK_DELAY] = delay;
                // TODO: SCAN_RSP_DELAY_VAL???
                // consider rx_done to ISR time, SW delay after read_current_fine_time(), func read_current_fine_time() delay ...
                delay = 118 - delay - SCAN_RSP_DELAY_VAL;  // IFS = 150us, Tx tail -> Rx done time: about 32us

                ll_hw_set_trx_settle(10, //LL_HW_BB_DELAY_VAL, //delay,             // set BB delay, about 80us in 16MHz HCLK                  
                                     LL_HW_AFE_DELAY_VAL, 
                                     LL_HW_PLL_DELAY_VAL);  //RxAFE,PLL    

                // reset Rx/Tx FIFO
                ll_hw_rst_rfifo();
                ll_hw_rst_tfifo(); 

                // TODO: Do we need this???
                // zigbee_crc16_gen(ackTxBuf+1, 3, seed, crcCode);
                // ackTxBuf[4] = crcCode[0];
                // ackTxBuf[5] = crcCode[1];
            
                // ll_hw_write_tfifo(ackTxBuf, 6);//include the crc16
                ll_hw_write_tfifo(ackTxBuf, 4); //no crc16
                rx_timestamps[SEND_ACK] = radio_timestamp();
                osThreadFlagsSet(m_config.threadid, RDFLG_ACK_START);
                ll_hw_go();
                //zb_hw_go();
            }
#endif       
            // Set timestamp
            // rxPkt->timestamp = ISR_entry_time;
            // rf_rxBuf = rf_rxBackupBuf;
            // if (rf_rxCbFunc)
            // {
            //     rf_rxCbFunc(rxPkt);
            // }

            // Set timestamp
            uint8_t len = packet.buffer[0];
						
						
						if (LL_HW_MODE_SRX == mode)
						{
								rf_setRxMode(MAX_RX_TIMEOUT);
						}
                        
            // if not ack
            if (len > 5 && ( dest1 == RADIO_BROADCAST_ADDR || (dest1 == m_config.nodeaddr) ) )
            {
                // TODO; use microseconds in the future
                // uint32_t airTimeUs = ((len + 1) * 8) * 4; // packet length / transmission speed
                
                rts = rts - 3; //airTimeUs;
                // Replace used CRC with timestamp
                packet.buffer[len - 1] = rts >> 24;
                packet.buffer[len] = rts >> 16;
                packet.buffer[len + 1] = rts >> 8;
                packet.buffer[len + 2] = rts;

                packet.rssi = -1 * zbRssi;
                
                // TODO: use linked list instead? it takes approx 300-400us to put msg in queue
                int res = osMessageQueuePut(m_config.recvQueue, &packet, 0, 0); //TODO packet len off by 2
                if(osOK != res)
                {
                    osThreadFlagsSet(m_config.threadid, RDFLG_QUEUE_ERROR);
                }
                else
                {
                    osThreadFlagsSet(m_config.threadid, RDFLG_RAIL_RX_SUCCESS);
                    rx_timestamps[RX_IRQ_FINISH] = radio_timestamp();
                }
            }
        }
    }
   
    // post ISR process   
		
		if(m_hw_stopping)
		{
			m_hw_stopping = false;
		}
		
    ll_hw_clr_irq();
    
    HAL_EXIT_CRITICAL_SECTION();
}

static comms_error_t radio_send (comms_layer_iface_t* interface, comms_msg_t* msg, comms_send_done_f* cb, void* user)
{
    comms_error_t err = COMMS_FAIL;
    
    debug2("rsnd: %p", msg);

    if (interface != (comms_layer_iface_t *)&m_radio_iface)
    {
        return (COMMS_EINVAL);
    }
    
    memset((void*)&tx_timestamps[0], 0x00, sizeof(tx_timestamps));
    tx_timestamps[START_RADIO_SEND] = radio_timestamp();
    
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
        // info3("snd %p \r\n", msg);
        err = COMMS_SUCCESS;
    }
    else
    {
        warn1("busy");
        err = COMMS_ENOMEM;
    }

    osMutexRelease(m_radio_mutex);
    
    debug2("snd %p e: %d", msg, err);
    return err;
}

/*
static void rf_wakeup_handler(void){
  NVIC_SetPriority((IRQn_Type)4, IRQ_PRIO_REALTIME);
  NVIC_SetPriority((IRQn_Type)20, IRQ_PRIO_HIGH);
}
*/

static void hal_rfphy_init (void)
{
    //========config the txPower
    // g_rfPhyTxPower  = RF_PHY_TX_POWER_EXTRA_MAX;
    g_rfPhyTxPower = RF_PHY_TX_POWER_5DBM;
    //============config BLE_PHY TYPE
    g_rfPhyPktFmt = PKT_FMT_ZIGBEE;
    //============config RF Frequency Offset
    g_rfPhyFreqOffSet = RF_PHY_FREQ_FOFF_20KHZ;

    XTAL16M_CAP_SETTING(0x09);
    XTAL16M_CURRENT_SETTING(0x01);

    hal_rom_boot_init();
    // TODO: check HIOT driver!
    NVIC_SetPriority((IRQn_Type)BB_IRQn, IRQ_PRIO_REALTIME);
    NVIC_SetPriority((IRQn_Type)TIM1_IRQn, IRQ_PRIO_HIGH);     //ll_EVT
    NVIC_SetPriority((IRQn_Type)TIM2_IRQn, IRQ_PRIO_HIGH);     //OSAL_TICK
    NVIC_SetPriority((IRQn_Type)TIM4_IRQn, IRQ_PRIO_HIGH);     //LL_EXA_ADV

    //hal_pwrmgr_register(MOD_USR0, NULL, rf_wakeup_handler);
    rf_phy_ini();
}

static comms_error_t radio_stop (comms_layer_iface_t* interface, comms_status_change_f* cb, void* user)
{
    comms_error_t err = COMMS_SUCCESS;
    if (interface != (comms_layer_iface_t *)&m_radio_iface)
    {
        return COMMS_EINVAL;
    }

    debug2("opst: %d", m_state);
        
    while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever));

    if (ST_OFF == m_state)
    {
        err = COMMS_ALREADY;
    }
    else if (ST_RUNNING != m_state)
    {
        err = COMMS_EBUSY;
    }
    else
    {
        m_state = ST_STOPPING;
        m_state_change_cb = cb;
        m_state_change_user = user;
        osThreadFlagsSet(m_config.threadid, RDFLG_RADIO_STOP);
    }
    // TODO: zb_hw_stop()?
    osMutexRelease(m_radio_mutex);
        
    return err;
}

static comms_error_t radio_start (comms_layer_iface_t* interface, comms_status_change_f* cb, void* user)
{
    comms_error_t err = COMMS_SUCCESS;
    if (interface != (comms_layer_iface_t *)&m_radio_iface)
    {
        return COMMS_EINVAL;
    }
        
    debug2("startst: %d", m_state);
        
    while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever));
        
    //debug1("State of radio is %d",m_state);
        
    if (ST_RUNNING == m_state)
    {
        err = COMMS_ALREADY;
    }
    else if (ST_OFF != m_state)
    {
        err = COMMS_EBUSY;
    }
    else
    {
        m_state = ST_STARTING;
        m_state_change_cb = cb;
        m_state_change_user = user;
        osThreadFlagsSet(m_config.threadid, RDFLG_RADIO_START);
    }
        
    osMutexRelease(m_radio_mutex);
        
    return err;
}

static void radio_send_timeout_cb (void* argument)
{
    osThreadFlagsSet(m_config.threadid, RDFLG_RADIO_SEND_TIMEOUT);
}

static void radio_resend_timeout_cb (void* argument)
{
    osThreadFlagsSet(m_config.threadid, RDFLG_RADIO_RESEND);
}

static void radio_ack_timeout_cb (void* arg)
{
    osThreadFlagsSet(m_config.threadid, RDFLG_RAIL_RXACK_TIMEOUT);
}

static void radio_ack_send_timeout_cb (void*arg)
{
    sending_ack = false;
    rf_setRxMode(MAX_RX_TIMEOUT);
    debug2("ACK send timeout");
    //osThreadFlagsSet(m_config.threadid, RDFLG_ACK_SENT_TIMEOUT);
}
    
static void stop_radio_now ()
{
    // debug1("stop");

    // Return any pending TX messages with COMMS_EOFF
    // No mutex, queue cannot change - send not accepting msgs in stop state
    while (NULL != radio_msg_queue_head)
    {
        radio_queue_element_t* qe = radio_msg_queue_head;
        warn1("rm txmsg %p", qe->msg);
        qe->send_done((comms_layer_t *)&m_radio_iface, qe->msg, COMMS_EOFF, qe->user);

        radio_msg_queue_head = qe->next;
        qe->next = radio_msg_queue_free;
        radio_msg_queue_free = qe;
    }
        
    data_pckt_t packet = {0};
    // Discard any pending RX messages
    while (osOK == osMessageQueueGet(m_config.recvQueue, &packet, NULL, 0))
    {
    }

    m_stop_timestamp = radio_timestamp();

    while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever))
    {
    }

    m_state = ST_OFF;
    osMutexRelease(m_radio_mutex);

    //zb_hw_stop();
        
    m_state_change_cb((comms_layer_t *)&m_radio_iface, COMMS_STOPPED, m_state_change_user);
}

static void start_radio_now ()
{
    //debug1("start");

    m_radio_channel_current = m_radio_channel_configured;

    while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever));

    m_state = ST_RUNNING;
    // radio_init should do it
    // phy_rf_rx();
    osMutexRelease(m_radio_mutex);

    m_state_change_cb((comms_layer_t *)&m_radio_iface, COMMS_STARTED, m_state_change_user);
}

void rf_tx (uint8_t* buf, uint8_t len, bool needAck, uint32_t evt_time)
{
    uint8_t cnt = 0;
    
    // uint8_t seed[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    // uint8_t crcCode[2]={0xff,0xff};
    
#ifdef LOG_MODE
    uint8_t mode;
    // check current radio mode
    mode = ll_hw_get_tr_mode();
    debug1("m_cfg:%u mode:%u", m_config.mode, mode);
#endif

    tx_timestamps[RF_TX] = radio_timestamp();
    
    // reset Rx/Tx FIFO
    ll_hw_rst_rfifo();
    ll_hw_rst_tfifo(); 
	
	
		uint32_t mode = ll_hw_get_tr_mode();

    if (LL_HW_MODE_STX != mode)
    {
        zb_hw_stop();
				zb_hw_set_stx();
    }
        
    if (evt_time != 0)
    {
        uint32_t diff, ti;
        uint16_t count = len - 19;
        ti = radio_timestamp();
        diff = evt_time - ti;

        buf[13+count] = diff>>24;
        buf[14+count] = diff>>16;
        buf[15+count] = diff>>8;
        buf[16+count] = diff;
    }

    ll_hw_write_tfifo(&buf[0], len);
    ll_hw_go();
    
    tx_timestamps[RF_TX_DONE] = radio_timestamp();
    
    m_radio_send_timestamp = radio_timestamp();
    transfer_pending = true;
    osTimerStart(m_send_timeout_timer, RADIO_MAX_SEND_TIME_MS);

    if (needAck)
    {
        osTimerStart(m_ack_timer, RADIO_WAIT_FOR_ACK_MS);
    }
}

static void radio_send_message (comms_msg_t * msg)
{
    __packed uint8_t buffer[160] = {0};

    if (NULL == msg)
    {
        sys_panic("snull");
    }
    
    tx_timestamps[RADIO_SEND_MSG] = radio_timestamp();
    
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
#ifdef USE_ACK
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
#else
    radio_tx_wait_ack = false;
    buffer[1] = 0x41;
#endif
    
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

    //info1("tx: %02X", buffer[12]);

    //	zb_hw_set_trx(0);

    uint32_t evt_time = 0;
    // Pick correct AMID, add timestamp footer when needed
    if (comms_event_time_valid(iface, msg))
    {
        uint32_t diff, ti;
        //debug1("evt time valid");
        buffer[11] = 0x3d; // 3D is used by TinyOS AM for timesync messages
        buffer[12+count] = amid;
        evt_time = comms_get_event_time(iface, msg);
        count += 5;
    }
    else
    {
        //debug1("evt time NOT valid");
        buffer[11] = amid;
    }

    buffer[0] = 11 + count + 2; // hdr, data, crc
    total = 1 + 11 + count + 2; // lenb, hdr, data, crc		

    //ll_hw_write_tfifo(&buffer[0], total);
    
    tx_timestamps[RADIO_SEND_MSG_PCKT_DONE] = radio_timestamp();
        
    if (checkEther() == PHY_CCA_IDLE)
    {
        rf_tx(buffer, total, radio_tx_wait_ack, evt_time);
    }
    else
    {
        // ll_hw_go();
        osThreadFlagsSet(m_config.threadid, RDFLG_RAIL_SEND_BUSY);
    }
        
}

static void signal_send_done (comms_error_t err)
{
    comms_send_done_f * send_done = NULL;
    comms_msg_t * msgp;
    void * user;
    uint32_t qtime;

    tx_timestamps[SIGNAL_SEND_DONE] = radio_timestamp();

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
        comms_set_timestamp((comms_layer_t *)&m_radio_iface, msgp, radio_timestamp()); // TODO: Crashes
        //comms_set_timestamp_us((comms_layer_t *)&m_radio_iface, msgp, radio_timestamp()); // TODO: Crashes
        //_comms_set_ack_received((comms_layer_t *)&m_radio_iface, msgp);
    }

    if (qtime > 15)
    {
        warn3("slow tx %d", qtime);
    }
    
    // logger(err==COMMS_SUCCESS?LOG_INFO3:LOG_WARN3,
    //       "snt %p e:%d t:(%"PRIu32")(%"PRIu32")",
    //        msgp, err,
    //        qtime,
    //        m_rail_sent_timestamp - m_rail_send_timestamp);

    //assert(NULL != send_done);

    //info1("snt");
    debug2("elapsed:%u cnt:%u", max_fine_time, max_carr_cnt);
    debug2("snt: %p %u", msgp, osKernelGetTickCount());

    // check how much hw_stop() take time
    if (m_hw_stop_end != m_hw_stop_start)
    {
        if (m_hw_stop_end > m_hw_stop_start)
        {
            warn1("hwstop:%u", m_hw_stop_end - m_hw_stop_start);
        }
        else
        {
            warn1("hwstop:%u", m_hw_stop_start - m_hw_stop_end);
        }
    }
    // phy_rf_rx();
    send_done((comms_layer_t *)&m_radio_iface, msgp, err, user);

#ifdef  LOG_TX_TIMESTAMPS    
    for (uint8_t idx = 0; idx < TX_LAST_TIMESTAMP; ++idx)
    {
        debug1("ts[%d]=%u", idx, tx_timestamps[idx]);
    }
#endif
}

static void radio_resend ()
{
    comms_msg_t * msg = NULL;

    while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever));

    uint8_t retu = comms_get_retries_used((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg) + 1;
    comms_set_retries_used((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg, retu);
    msg = radio_msg_sending->msg;
    osMutexRelease(m_radio_mutex);

    if (NULL != msg)
    {
        radio_send_message(msg);
    }
}

static void handle_radio_tx (uint32_t flags)
{
    // If sending, see if it has completed
    if (NULL != radio_msg_sending)
    {
        // Sending has completed -----------------------------------------------
        if (flags & RDFLG_RAIL_SEND_DONE)
        {
            update_tx_stats(radio_msg_sending->msg);

            if (radio_tx_wait_ack) // Alternatively we should get rx_ack_timeout
            {
                debug2("ackd");
                osTimerStop(m_ack_timer);
                radio_tx_wait_ack = false;
                _comms_set_ack_received((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg);
            }
            signal_send_done(COMMS_SUCCESS);
        }
        // Ack was not received ------------------------------------------------
        else if (flags & RDFLG_RAIL_RXACK_TIMEOUT)
        {
            bool resend = false;

            osTimerStop(m_send_timeout_timer);

            //update_tx_stats(radio_msg_sending->msg);

            if (comms_get_retries_used((comms_layer_t*)&m_radio_iface, radio_msg_sending->msg) < \
                comms_get_retries((comms_layer_t*)&m_radio_iface, radio_msg_sending->msg))
            {
                resend = true;
            }

            // logger(resend?LOG_DEBUG1:LOG_WARN3, "rx ackTimeout (%"PRIu8"/%"PRIu8")",
            //        comms_get_retries_used((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg),
            //        comms_get_retries((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg));
            
            if (resend)
            {
                osTimerStart(m_resend_timer, \
                             comms_get_timeout((comms_layer_t *)&m_radio_iface, \
                             radio_msg_sending->msg));
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
            transfer_pending = false;

            //osTimerStop(m_send_timeout_timer);

            if (m_csma_retries < 7)
            {
                resend = true;
                m_csma_retries++;
            }

            if (resend)
            {
                radio_send_message(radio_msg_sending->msg);
                //radio_resend();
            }
            else
            {
                if (comms_get_retries_used((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg) < \
                    comms_get_retries((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg))
                {
                    m_csma_retries = 0;
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
            transfer_pending = false;
            signal_send_done(COMMS_FAIL);
        }
        // Sending has not completed in a reasonable amount of time ------------
        else if (flags & RDFLG_RADIO_SEND_TIMEOUT)
        {
            // Check that an actual timeout has happened and this is not race
            uint32_t passed = radio_timestamp() - m_radio_send_timestamp;
            if (passed >= RADIO_MAX_SEND_TIME_MS)
            {
                err1("TIMEOUT\r\n");
                transfer_pending = false;
                signal_send_done(COMMS_ETIMEOUT);

                // Presumably something is wrong with the radio
                osThreadFlagsSet(m_config.threadid, RDFLG_RADIO_RESTART);
            }
            else // Perhaps triggered because timer from previous send was not stopped in time
            {
                warn1("restarting timeout with new value\r\n");
                osTimerStart(m_send_timeout_timer, RADIO_MAX_SEND_TIME_MS - passed);
            }
        }
        // PacketLink resend functionality -------------------------------------
        else if (flags & RDFLG_RADIO_RESEND)
        {
            radio_resend();
        }
    }
}

static void handle_radio_rx ()
{
    //uint8_t buffer[140];
    data_pckt_t packet = {0};
    rx_timestamps[RX_PRC_START] = radio_timestamp();
    // RX processing -----------------------------------------------------------
    if (osOK == osMessageQueueGet(m_config.recvQueue, &packet, NULL, 0))
    {
        uint8_t len = packet.buffer[0];
        uint32_t rts = (packet.buffer[len-1] << 24) | (packet.buffer[len] << 16) | (packet.buffer[len+1] << 8) | packet.buffer[len+2];

        if ((packet.buffer[2] == 0x88) && (packet.buffer[5] == 0x00) && (packet.buffer[10] == 0x3F))
        {
            comms_msg_t msg;
            am_id_t amid;
            void* payload;
            uint8_t plen = len -13;
            uint8_t lqi = 0xFF;
            uint16_t source = ((uint16_t)packet.buffer[8] << 0) | ((uint16_t)packet.buffer[9] << 8);
            uint32_t currTime = radio_timestamp();
            uint32_t timestamp = currTime - (currTime - rts);

            comms_init_message((comms_layer_t *)&m_radio_iface, &msg);

            if (packet.buffer[11] == 0x3D)
            {
                int32_t diff = (packet.buffer[len - 5] << 24) | (packet.buffer[len - 4] << 16) | (packet.buffer[len- 3] << 8) | (packet.buffer[len- 2]);

                if ((len < 17) || (len > 200))
                {
                        sys_panic("packet");
                }
                amid = packet.buffer[(len-6)];
                plen = len - 18;
                
                //debug1("diff: %i, amid: %02x", diff, amid);
                //comms_set_event_time_us((comms_layer_t *)&m_radio_iface, &msg, (uint32_t)(diff + timestamp));
                comms_set_event_time((comms_layer_t *)&m_radio_iface, &msg, (uint32_t)(diff + timestamp));
                //debug1("rts: %d, currTime: %d, timestamp: %d, diff: %d", rts, currTime, timestamp, diff);
            }
            else
            {
                amid = packet.buffer[11];
                //debug1("amid: %02x", amid);
                //plen = packetInfo.packetBytes - 12;
            }

            payload = comms_get_payload((comms_layer_t *)&m_radio_iface, &msg, plen);

            if (NULL != payload)
            {
                uint16_t dest = ((uint16_t)packet.buffer[6] << 0) | ((uint16_t)packet.buffer[7] << 8);

                comms_set_packet_type((comms_layer_t *)&m_radio_iface, &msg, amid);
                comms_set_payload_length((comms_layer_t *)&m_radio_iface, &msg, plen);
                memcpy(payload, (const void *)&packet.buffer[12], plen);

                comms_set_timestamp((comms_layer_t *)&m_radio_iface, &msg, rts);
                //comms_set_timestamp_us((comms_layer_t *)&m_radio_iface, &msg, rts);

                if (ST_OFF != m_state)
                {
                    debug2("rx: %02X a:%02X", packet.buffer[12],packet.buffer[1]);
                }
                //debug2("rx: %02X a:%02X", packet.buffer[12],packet.buffer[1]);

                int16_t rssi = packet.rssi;
                _comms_set_rssi((comms_layer_t *)&m_radio_iface, &msg, rssi);
                if (rssi < -96)
                {
                    lqi = 0;
                }
                else if (rssi < -93) // RFR2-like LQI simulation
                {
                    lqi = lqi + (rssi+93)*50;
                }

                _comms_set_lqi((comms_layer_t *)&m_radio_iface, &msg, lqi);
                comms_am_set_destination((comms_layer_t *)&m_radio_iface, &msg, dest);
                comms_am_set_source((comms_layer_t *)&m_radio_iface, &msg, source);

                comms_deliver((comms_layer_t *)&m_radio_iface, &msg);
                rx_timestamps[RX_PRC_END] = radio_timestamp();

                #ifdef  LOG_RX_TIMESTAMPS    
                  for (uint8_t idx = 0; idx < RX_LAST_TIMESTAMP; ++idx)
                  {
                    debug1("ts[%d]=%u", idx, rx_timestamps[idx]);
                  }
                #endif
            }
        }
    }
}

static void radio_send_next ()
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
        radio_send_message(msg);
    }
}

static void handle_radio_events (uint32_t flags)
{
    if (flags & RDFLG_RAIL_RX_OVERFLOW)
    {
        ll_hw_rst_rfifo();
    }

    if (flags & RDFLG_RAIL_RX_BUSY)
    {

    }

    if (flags & RDFLG_RAIL_RXACK_TIMEOUT)
    {

    }

    if (flags & RDFLG_RAIL_RX_SUCCESS)
    {
         //debug1("Rx");
    }
        
    if (flags & RDFLG_CRC_ERROR)
    {
        // warn1("CRC error");
    }

    if (flags & RDFLG_RAIL_TXACK_SENT)
    {
        debug2("ACK snt");
        osTimerStop(m_ack_timeout_timer);
    }
    
    if (flags & RDFLG_ACK_START)
    {
        if (sending_ack)
        {
           // debug1("Ack tmr strt");
            osTimerStart(m_ack_timeout_timer, RADIO_WAIT_FOR_ACK_SENT_MS);
        }
    }

    if (flags & RDFLG_QUEUE_ERROR)
    {
        err1("Failed to put message in queue!");
    }
    
    // if ((flags & RDFLG_RAIL_TXACK_SENT) || (flags & RDFLG_RAIL_RX_SUCCESS))
    // {
    //     phy_rf_rx();
    // }
        
//    if (flags & RDFLG_RADIO_STRT_ACK_TIM)
//    {
//        osTimerStart(m_ack_timer, RADIO_WAIT_FOR_ACK_MS);
//    }
    
    // if (flags & RDFLG_RADIO_ACK)
    // {
    //     debug1("Sending ack");
    //     send_ack_packet();
    // }
}


static void radio_task (void* arg)
{
    bool running = false;
    bool led_set = false;
    // uint32_t flags = osThreadFlagsWait(RDFLGS_ALL, osFlagsWaitAny, osWaitForever);
    uint32_t state;
    uint32_t flags = osFlagsErrorTimeout;

    //debug1("Thread starts");
    flags = osThreadFlagsClear(RDFLGS_ALL);
   // debug1("ThrFlgs:0x%X", flags);

    for (;;)
    {

        flags = osThreadFlagsWait(RDFLGS_ALL, osFlagsWaitAny, osWaitForever);

        // debug1("ThrFlgs:0x%X", flags);
        if (flags & ~RDFLGS_ALL)
        {
            err1("ThrdError:%X", flags);
            continue;
        }
    
        while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever));
        state = m_state;
        osMutexRelease(m_radio_mutex);
         
        if (ST_STARTING == state)
        {
            //debug1("Starting radio now");
            start_radio_now();
            running = true;
        }
        
        // If an exception has occurred and RAIL is broken ---------------------
        if (flags & RDFLG_RADIO_RESTART)
        {
           // debug1("restart");
            zb_hw_stop();

            // If sending, cancel and notify user
            if (NULL != radio_msg_sending)
            {
                flags |= RDFLG_RADIO_SEND_FAIL;
            }
        }

        // Handle TX activities
        handle_radio_tx(flags);

        // Check RX queue and process any messages there
        handle_radio_rx(); //TODO: uncomment

        // Handle "other" events
        handle_radio_events(flags);

        if (radio_msg_sending == NULL)
        {
            if (ST_STOPPING == state)
            {
                stop_radio_now();
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
        
        // toggle LED
        if (true == led_set)
        {
            hal_gpio_write(GPIO_P25, 0);
            led_set = false;
        }
        else
        {
            hal_gpio_write(GPIO_P25, 1);
            led_set = true;
        }
    }
}

radio_config_t* init_radio (uint16_t nodeaddr, uint8_t channel, uint8_t pan)
{
    if(channel < 10 || channel > 26)
    {
        return NULL;
    }

    m_config.channel = channel;

    m_config.nodeaddr = nodeaddr;
    m_config.pan = pan;
    m_config.cca_treshhold = 70;

    int res = comms_am_create((comms_layer_t *)&m_radio_iface, nodeaddr, radio_send, radio_start, radio_stop);

    if(COMMS_SUCCESS != res)
    {
        return NULL;
    }

    JUMP_FUNCTION(V4_IRQ_HANDLER) = (uint32_t)&RFPHY_IRQHandler;

    m_config.recvQueue = osMessageQueueNew(10, 142, NULL);
    m_config.radio = (comms_layer_t *)&m_radio_iface;

    if(NULL == m_config.recvQueue)
    {
        return NULL;
    }

    const osThreadAttr_t main_thread_attr = 
    {
        .name = "radio",
        .priority =  osPriorityHigh // osPriorityNormal osPriorityHigh osPriorityAboveNormal
    };

    m_config.threadid = osThreadNew(radio_task, NULL, &main_thread_attr);

    if(NULL == m_config.threadid)
    {
        return NULL;
    }

    radio_msg_sending = NULL;
    radio_msg_queue_head = NULL;
    radio_msg_queue_free = &radio_msg_queue_memory[0];
    radio_msg_queue_free->next = NULL;
    for (uint8_t i = 1; i < sizeof(radio_msg_queue_memory) / sizeof(radio_queue_element_t); i++)
    {
        radio_msg_queue_memory[i].next = (radio_queue_element_t*)radio_msg_queue_free;
        radio_msg_queue_free = &radio_msg_queue_memory[i];
    }

    m_radio_mutex = osMutexNew(NULL);
    m_send_timeout_timer = osTimerNew(&radio_send_timeout_cb, osTimerOnce, NULL, NULL);
    m_resend_timer = osTimerNew(&radio_resend_timeout_cb, osTimerOnce, NULL, NULL);
    m_ack_timer = osTimerNew(&radio_ack_timeout_cb, osTimerOnce, NULL, NULL);
    m_ack_timeout_timer = osTimerNew(&radio_ack_send_timeout_cb, osTimerOnce, NULL, NULL);

    m_state = ST_OFF;

    memset(&m_rxBuf[0], 0x0, 127);

    hal_rfphy_init();
    zb_set_channel(channel);
    zb_hw_timing ();
    //rf_setRxMode(MAX_RX_TIMEOUT);
		zb_hw_set_srx(MAX_RX_TIMEOUT);
    // reset Rx/Tx FIFO
    ll_hw_rst_rfifo();
    ll_hw_rst_tfifo();
    ll_hw_go();
		

    return &m_config;
}

void radio_deinit (comms_layer_t* iface)
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

uint32_t radio_tx_packets()
{
    return m_transmitted_packets;
}

uint32_t radio_tx_bytes()
{
    return m_transmitted_bytes;
}

