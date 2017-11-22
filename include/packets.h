#ifndef PACKETS_H
#define PACKETS_H

#ifdef DP100
#define SOF_RX '>'
#define SOF_TX '<'
#else
#define SOF_RX '<'
#define SOF_TX '>'
#endif

#define SEPARATOR ','

#define DISPLAY_ID_POS 5

static char regDispCmdResp[33] = { SOF_TX, SEPARATOR, '8','3', SEPARATOR, '0', SEPARATOR,
          '0','0','0','0','0','0','0','0','-',
          '0','0','0','0','0','0','0','0','-',
          '0','0','0','0','0','0','0','0'};

/**
 * ACK Packet - Sent in response to any led set commands
 * Contains SOF, CMD, ID (obtained at registration)
 */
static char dispCmdAck[7] = { SOF_TX, SEPARATOR, '9', '9', SEPARATOR, '0', SEPARATOR };

#endif
