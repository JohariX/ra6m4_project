/* generated vector header file - do not edit */
        #ifndef VECTOR_DATA_H
        #define VECTOR_DATA_H
        #ifdef __cplusplus
        extern "C" {
        #endif
                /* Number of interrupts allocated */
        #ifndef VECTOR_DATA_IRQ_COUNT
        #define VECTOR_DATA_IRQ_COUNT    (13)
        #endif
        /* ISR prototypes */
        void sci_uart_rxi_isr(void);
        void sci_uart_txi_isr(void);
        void sci_uart_tei_isr(void);
        void sci_uart_eri_isr(void);
        void can_error_isr(void);
        void can_rx_isr(void);
        void can_tx_isr(void);

        /* Vector table allocations */
        #define VECTOR_NUMBER_SCI9_RXI ((IRQn_Type) 0) /* SCI9 RXI (Receive data full) */
        #define SCI9_RXI_IRQn          ((IRQn_Type) 0) /* SCI9 RXI (Receive data full) */
        #define VECTOR_NUMBER_SCI9_TXI ((IRQn_Type) 1) /* SCI9 TXI (Transmit data empty) */
        #define SCI9_TXI_IRQn          ((IRQn_Type) 1) /* SCI9 TXI (Transmit data empty) */
        #define VECTOR_NUMBER_SCI9_TEI ((IRQn_Type) 2) /* SCI9 TEI (Transmit end) */
        #define SCI9_TEI_IRQn          ((IRQn_Type) 2) /* SCI9 TEI (Transmit end) */
        #define VECTOR_NUMBER_SCI9_ERI ((IRQn_Type) 3) /* SCI9 ERI (Receive error) */
        #define SCI9_ERI_IRQn          ((IRQn_Type) 3) /* SCI9 ERI (Receive error) */
        #define VECTOR_NUMBER_SCI0_RXI ((IRQn_Type) 4) /* SCI0 RXI (Receive data full) */
        #define SCI0_RXI_IRQn          ((IRQn_Type) 4) /* SCI0 RXI (Receive data full) */
        #define VECTOR_NUMBER_SCI0_TXI ((IRQn_Type) 5) /* SCI0 TXI (Transmit data empty) */
        #define SCI0_TXI_IRQn          ((IRQn_Type) 5) /* SCI0 TXI (Transmit data empty) */
        #define VECTOR_NUMBER_SCI0_TEI ((IRQn_Type) 6) /* SCI0 TEI (Transmit end) */
        #define SCI0_TEI_IRQn          ((IRQn_Type) 6) /* SCI0 TEI (Transmit end) */
        #define VECTOR_NUMBER_SCI0_ERI ((IRQn_Type) 7) /* SCI0 ERI (Receive error) */
        #define SCI0_ERI_IRQn          ((IRQn_Type) 7) /* SCI0 ERI (Receive error) */
        #define VECTOR_NUMBER_CAN0_ERROR ((IRQn_Type) 8) /* CAN0 ERROR (Error interrupt) */
        #define CAN0_ERROR_IRQn          ((IRQn_Type) 8) /* CAN0 ERROR (Error interrupt) */
        #define VECTOR_NUMBER_CAN0_MAILBOX_RX ((IRQn_Type) 9) /* CAN0 MAILBOX RX (Reception complete interrupt) */
        #define CAN0_MAILBOX_RX_IRQn          ((IRQn_Type) 9) /* CAN0 MAILBOX RX (Reception complete interrupt) */
        #define VECTOR_NUMBER_CAN0_MAILBOX_TX ((IRQn_Type) 10) /* CAN0 MAILBOX TX (Transmission complete interrupt) */
        #define CAN0_MAILBOX_TX_IRQn          ((IRQn_Type) 10) /* CAN0 MAILBOX TX (Transmission complete interrupt) */
        #define VECTOR_NUMBER_CAN0_FIFO_RX ((IRQn_Type) 11) /* CAN0 FIFO RX (Receive FIFO interrupt) */
        #define CAN0_FIFO_RX_IRQn          ((IRQn_Type) 11) /* CAN0 FIFO RX (Receive FIFO interrupt) */
        #define VECTOR_NUMBER_CAN0_FIFO_TX ((IRQn_Type) 12) /* CAN0 FIFO TX (Transmit FIFO interrupt) */
        #define CAN0_FIFO_TX_IRQn          ((IRQn_Type) 12) /* CAN0 FIFO TX (Transmit FIFO interrupt) */
        /* The number of entries required for the ICU vector table. */
        #define BSP_ICU_VECTOR_NUM_ENTRIES (13)

        #ifdef __cplusplus
        }
        #endif
        #endif /* VECTOR_DATA_H */
