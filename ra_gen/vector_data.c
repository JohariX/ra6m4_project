/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = sci_uart_rxi_isr, /* SCI9 RXI (Receive data full) */
            [1] = sci_uart_txi_isr, /* SCI9 TXI (Transmit data empty) */
            [2] = sci_uart_tei_isr, /* SCI9 TEI (Transmit end) */
            [3] = sci_uart_eri_isr, /* SCI9 ERI (Receive error) */
            [4] = sci_uart_rxi_isr, /* SCI0 RXI (Receive data full) */
            [5] = sci_uart_txi_isr, /* SCI0 TXI (Transmit data empty) */
            [6] = sci_uart_tei_isr, /* SCI0 TEI (Transmit end) */
            [7] = sci_uart_eri_isr, /* SCI0 ERI (Receive error) */
            [8] = can_error_isr, /* CAN0 ERROR (Error interrupt) */
            [9] = can_rx_isr, /* CAN0 MAILBOX RX (Reception complete interrupt) */
            [10] = can_tx_isr, /* CAN0 MAILBOX TX (Transmission complete interrupt) */
            [11] = can_rx_isr, /* CAN0 FIFO RX (Receive FIFO interrupt) */
            [12] = can_tx_isr, /* CAN0 FIFO TX (Transmit FIFO interrupt) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_SCI9_RXI,GROUP0), /* SCI9 RXI (Receive data full) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_SCI9_TXI,GROUP1), /* SCI9 TXI (Transmit data empty) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_SCI9_TEI,GROUP2), /* SCI9 TEI (Transmit end) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_SCI9_ERI,GROUP3), /* SCI9 ERI (Receive error) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_SCI0_RXI,GROUP4), /* SCI0 RXI (Receive data full) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_SCI0_TXI,GROUP5), /* SCI0 TXI (Transmit data empty) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_SCI0_TEI,GROUP6), /* SCI0 TEI (Transmit end) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_SCI0_ERI,GROUP7), /* SCI0 ERI (Receive error) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_CAN0_ERROR,GROUP0), /* CAN0 ERROR (Error interrupt) */
            [9] = BSP_PRV_VECT_ENUM(EVENT_CAN0_MAILBOX_RX,GROUP1), /* CAN0 MAILBOX RX (Reception complete interrupt) */
            [10] = BSP_PRV_VECT_ENUM(EVENT_CAN0_MAILBOX_TX,GROUP2), /* CAN0 MAILBOX TX (Transmission complete interrupt) */
            [11] = BSP_PRV_VECT_ENUM(EVENT_CAN0_FIFO_RX,GROUP3), /* CAN0 FIFO RX (Receive FIFO interrupt) */
            [12] = BSP_PRV_VECT_ENUM(EVENT_CAN0_FIFO_TX,GROUP4), /* CAN0 FIFO TX (Transmit FIFO interrupt) */
        };
        #endif
        #endif
