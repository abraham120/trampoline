/* FlexCAN.h              (c) 2015 Freescale Semiconductor, Inc.
 * Descriptions: FTM example code.
 * 16 Sep 2016 SM: Initial version
 */


#ifndef FLEXCAN_H_
#define FLEXCAN_H_

#define NODE_A        /* If using 2 boards as 2 nodes, NODE A & B use different CAN IDs */
#define SBC_MC33903   /* SBC requires SPI init + max 1MHz bit rate */

// ADD by JYM 2017.07.12
#define DK_AUTOSAR    /* DK-AUTOSAR or S32K144 */
#define STANDARD_CAN_ID_SHIFT 18

void FLEXCAN0_init (void);

#ifdef DK_AUTOSAR
void FLEXCAN0_transmit_msg(int id, uint32_t high_data, uint32_t low_data);

#else
void FLEXCAN0_transmit_msg (void);
#endif

#endif /* FLEXCAN_H_ */
