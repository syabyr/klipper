// CANbus support on SAM4E chips
//
// Copyright (C) 2021-2025  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 syabyr
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/irq.h" // irq_save
#include "command.h" // DECL_CONSTANT_STR
#include "generic/armcm_boot.h" // armcm_enable_irq
#include "generic/canbus.h" // canbus_notify_tx
#include "generic/canserial.h" // CANBUS_ID_ADMIN
#include "internal.h" // enable_pclock
#include "sched.h" // DECL_INIT

#include <sam4e8e.h>

/****************************************************************
 * Pin configuration
 ****************************************************************/

#if CONFIG_ATSAM_CANBUS_PB3_PB2
 DECL_CONSTANT_STR("RESERVE_PINS_CAN", "PB3,PB2");
 #define GPIO_Rx GPIO('B', 3)
 #define GPIO_Tx GPIO('B', 2)
 #define CANx CAN0
 #define CANx_IRQn CAN0_IRQn
 #define CANx_GCLK_ID ID_CAN0
 #define AF_Rx 'A'
 #define AF_Tx 'A'
#elif CONFIG_ATSAM_CANBUS_PC12_PC15
 DECL_CONSTANT_STR("RESERVE_PINS_CAN", "PC12,PC15");
 #define GPIO_Rx GPIO('C', 12)
 #define GPIO_Tx GPIO('C', 15)
 #define CANx CAN1
 #define CANx_IRQn CAN1_IRQn
 #define CANx_GCLK_ID ID_CAN1
 #define AF_Rx 'C'
 #define AF_Tx 'C'
#endif

/****************************************************************
 * CANbus code
 ****************************************************************/

// Maximum mailbox number (SAM4E has 8 mailboxes)
#define CAN_MB_COUNT 8

// Transmit mailbox - we use one mailbox for transmission
#define CAN_TX_MB 0
// First receive mailbox
#define CAN_RX_MB_START 1

static struct {
    uint32_t rx_error, tx_error;
} CAN_Errors;

// Report interface status
void
canhw_get_status(struct canbus_status *status)
{
    irqstatus_t flag = irq_save();
    uint32_t ecr = CANx->CAN_ECR;
    uint32_t sr = CANx->CAN_SR;
    uint32_t rx_error = CAN_Errors.rx_error, tx_error = CAN_Errors.tx_error;
    irq_restore(flag);

    status->rx_error = rx_error;
    status->tx_error = tx_error;
    if (sr & CAN_SR_BOFF)
        status->bus_state = CANBUS_STATE_OFF;
    else if (sr & CAN_SR_ERRP)
        status->bus_state = CANBUS_STATE_PASSIVE;
    else if (sr & CAN_SR_WARN)
        status->bus_state = CANBUS_STATE_WARN;
    else
        status->bus_state = 0;
}

// Setup the receive packet filter
void
canhw_set_filter(uint32_t id)
{
    // Setup acceptance filters in mailboxes
    if (!CONFIG_CANBUS_FILTER)
        return;

    // Filter for CANBUS_ID_ADMIN
    CANx->CAN_MB[CAN_RX_MB_START].CAN_MAM = (CANBUS_ID_ADMIN & 0x7FF) << 18;
    CANx->CAN_MB[CAN_RX_MB_START].CAN_MID = (CANBUS_ID_ADMIN & 0x7FF) << 18;
    CANx->CAN_MB[CAN_RX_MB_START].CAN_MMR =
        (CAN_MMR_MOT_MB_RX << CAN_MMR_MOT_Pos);

    // Filter for id
    CANx->CAN_MB[CAN_RX_MB_START + 1].CAN_MAM = (id & 0x7FF) << 18;
    CANx->CAN_MB[CAN_RX_MB_START + 1].CAN_MID = (id & 0x7FF) << 18;
    CANx->CAN_MB[CAN_RX_MB_START + 1].CAN_MMR =
        (CAN_MMR_MOT_MB_RX << CAN_MMR_MOT_Pos);

    // Filter for id + 1
    CANx->CAN_MB[CAN_RX_MB_START + 2].CAN_MAM = ((id + 1) & 0x7FF) << 18;
    CANx->CAN_MB[CAN_RX_MB_START + 2].CAN_MID = ((id + 1) & 0x7FF) << 18;
    CANx->CAN_MB[CAN_RX_MB_START + 2].CAN_MMR =
        (CAN_MMR_MOT_MB_RX << CAN_MMR_MOT_Pos);
}

// Transmit a packet
int
canhw_send(struct canbus_msg *msg)
{
    // Check if transmit mailbox is ready
    if (!(CANx->CAN_SR & (1 << CAN_TX_MB))) {
        // Mailbox busy - wait for irq
        return -1;
    }

    uint32_t ids;
    if (msg->id & CANMSG_ID_EFF)
        ids = ((msg->id & 0x1fffffff) << 0) | CAN_MID_MIDE;
    else
        ids = (msg->id & 0x7ff) << 18;
    ids |= msg->id & CANMSG_ID_RTR ? CAN_MSR_MRTR : 0;

    // Set ID and data
    CANx->CAN_MB[CAN_TX_MB].CAN_MID = ids;
    uint32_t data[2];
    memcpy(data, msg->data, sizeof(data));
    CANx->CAN_MB[CAN_TX_MB].CAN_MDL = data[0];
    CANx->CAN_MB[CAN_TX_MB].CAN_MDH = data[1];
    CANx->CAN_MB[CAN_TX_MB].CAN_MCR = (msg->dlc & 0x0f) << CAN_MCR_MDLC_Pos;

    // Trigger transmission
    CANx->CAN_TCR = 1 << CAN_TX_MB;

    return CANMSG_DATA_LEN(msg);
}

// This function handles CAN global interrupts
void
CAN_IRQHandler(void)
{
    uint32_t imr = CANx->CAN_IMR;
    uint32_t sr = CANx->CAN_SR;

    // Check for receive
    for (int mb = CAN_RX_MB_START; mb < CAN_MB_COUNT; mb++) {
        if ((sr & (1 << mb)) && (imr & (1 << mb))) {
            CanMb *mb_ptr = &CANx->CAN_MB[mb];
            uint32_t mid = mb_ptr->CAN_MID;
            uint32_t msr = mb_ptr->CAN_MSR;

            struct canbus_msg msg;
            if (mid & CAN_MID_MIDE) {
                msg.id = mid & 0x1fffffff;
                msg.id |= CANMSG_ID_EFF;
            } else {
                msg.id = (mid >> 18) & 0x7ff;
            }
            if (msr & CAN_MSR_MRTR)
                msg.id |= CANMSG_ID_RTR;
            msg.dlc = (msr & CAN_MSR_MDLC_Msk) >> CAN_MSR_MDLC_Pos;

            uint32_t mdl = mb_ptr->CAN_MDL;
            uint32_t mdh = mb_ptr->CAN_MDH;
            msg.data32[0] = mdl;
            msg.data32[1] = mdh;

            // Process packet
            canbus_process_data(&msg);

            // Clear mailbox
            mb_ptr->CAN_MCR = 0;
        }
    }

    // Check for transmit complete
    if (sr & (1 << CAN_TX_MB) && (imr & (1 << CAN_TX_MB))) {
        // Transmit done - clear status
        CANx->CAN_MB[CAN_TX_MB].CAN_MCR = 0;
        canbus_notify_tx();
    }

    // Check for bus errors
    if (sr & CAN_SR_CERR && (imr & CAN_IER_CERR))
        CAN_Errors.rx_error++;
    if (sr & CAN_SR_SERR && (imr & CAN_IER_SERR))
        CAN_Errors.rx_error++;
    if (sr & CAN_SR_AERR && (imr & CAN_IER_AERR))
        CAN_Errors.rx_error++;
    if (sr & CAN_SR_FERR && (imr & CAN_IER_FERR))
        CAN_Errors.rx_error++;
    if (sr & CAN_SR_BERR && (imr & CAN_IER_BERR))
        CAN_Errors.tx_error++;
}

static inline const uint32_t
make_br(uint32_t sjw,       // Sync jump width
         uint32_t phase2,   // Phase 2 segment
         uint32_t phase1,   // Phase 1 segment
         uint32_t propag,   // Propagation segment
         uint32_t brp)       // Baud rate prescaler
{
    return (((uint32_t)(sjw-1)) << CAN_BR_SJW_Pos
            | ((uint32_t)(phase2-1)) << CAN_BR_PHASE2_Pos
            | ((uint32_t)(phase1-1)) << CAN_BR_PHASE1_Pos
            | ((uint32_t)(propag-1)) << CAN_BR_PROPAG_Pos
            | ((uint32_t)(brp - 1)) << CAN_BR_BRP_Pos);
}

static inline const uint32_t
compute_br(uint32_t pclock, uint32_t bitrate)
{
    /*
        Bit timing:
        Total bit time = (1 + phase1 + phase2 + propag) * (brp + 1) / pclock
        We want sample point around 87.5%
     */
    uint32_t total_tq = pclock / bitrate;

    // Find best division
    uint32_t sjw = 2;
    uint32_t propag, phase1, phase2, brp;

    // Try to get total_tq between 8 and 25
    for (brp = 1; brp <= 128; brp++) {
        if (total_tq % brp == 0) {
            uint32_t qs = total_tq / brp;
            if (qs >= 5 && qs <= 20) {
                // 1 (sync) + propag + phase1 + phase2 = qs
                // Sample at 87.5% => phase2 = qs/8, propag+phase1 = qs - 1 - phase2
                phase2 = qs / 8;
                propag = 1;
                phase1 = qs - 1 - phase2 - propag;
                if (phase1 >= 1 && phase1 <= 7 && phase2 >= 1 && phase2 <= 7)
                    break;
            }
        }
    }
    if (brp > 128) {
        // Fallback - find any working combination
        brp = total_tq / 10;
        uint32_t qs = total_tq / brp;
        phase2 = qs / 8;
        propag = 1;
        phase1 = qs - 1 - phase2 - propag;
        if (phase1 > 7) {
            phase1 = 7;
            phase2 = qs - 1 - propag - phase1;
        }
    }

    return make_br(sjw, phase2, phase1, propag, brp);
}

void
can_init(void)
{
    // Enable peripheral clock
    enable_pclock(CANx_GCLK_ID);

    // Configure pins
    gpio_peripheral(GPIO_Rx, AF_Rx, 1);
    gpio_peripheral(GPIO_Tx, AF_Tx, 0);

    // Get peripheral clock frequency
    uint32_t pclock = get_pclock_frequency(CANx_GCLK_ID);

    // Compute baud rate setting
    uint32_t br = compute_br(pclock, CONFIG_CANBUS_FREQUENCY);

    // Enable CAN controller
    CANx->CAN_MR = CAN_MR_CANEN;
    CANx->CAN_BR = br | CAN_BR_SMP_THREE; // Sample three times

    // Configure mailboxes
    for (int i = 0; i < CAN_MB_COUNT; i++) {
        CANx->CAN_MB[i].CAN_MMR = 0; // Disable mailbox
    }

    // Configure transmit mailbox
    CANx->CAN_MB[CAN_TX_MB].CAN_MMR = (CAN_MMR_MOT_MB_TX << CAN_MMR_MOT_Pos);

    // Setup filter
    canhw_set_filter(0);

    // Enable interrupts
    armcm_enable_irq(CAN_IRQHandler, CANx_IRQn, 1);

    // Enable all error interrupts and mailbox interrupts
    uint32_t ier = 0;
    for (int i = 0; i < CAN_MB_COUNT; i++) {
        ier |= 1 << i;
    }
    ier |= CAN_IER_ERRA | CAN_IER_WARN | CAN_IER_ERRP | CAN_IER_BOFF
        | CAN_IER_CERR | CAN_IER_SERR | CAN_IER_AERR | CAN_IER_FERR | CAN_IER_BERR;
    CANx->CAN_IER = ier;
}
DECL_INIT(can_init);
