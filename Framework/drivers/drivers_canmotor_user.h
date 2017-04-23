#ifndef DRIVERS_CANMOTOR_USER_H
#define DRIVERS_CANMOTOR_USER_H

#include "utilities_iopool.h"
#include "can.h"

//RxID
#define CMFL_RXID 0x201u
#define CMFR_RXID 0x202u
#define CMBL_RXID 0x203u
#define CMBR_RXID 0x204u

#define GMYAW_RXID 0x205u
#define GMPITCH_RXID 0x206u

#define AMUDFL_RXID 0x201u
#define AMUDFR_RXID 0x202u
#define AMUDBL_RXID 0x203u
#define AMUDBR_RXID 0x204u
#define AMPLATE_RXID 0x205u
#define AMGETBULLET_RXID 0x206u

//TxID
#define CM_TXID 0x200u
#define GM_TXID 0x1FFu
#define AM1_TXID 0x200u
#define AM2_TXID 0x1FFu

//RxIOPool
IOPoolDeclare(CMFLRxIOPool, CanRxMsgTypeDef);
IOPoolDeclare(CMFRRxIOPool, CanRxMsgTypeDef);
IOPoolDeclare(CMBLRxIOPool, CanRxMsgTypeDef);
IOPoolDeclare(CMBRRxIOPool, CanRxMsgTypeDef);

IOPoolDeclare(GMPITCHRxIOPool, CanRxMsgTypeDef);
IOPoolDeclare(GMYAWRxIOPool, CanRxMsgTypeDef);

IOPoolDeclare(AMUDFLRxIOPool, CanRxMsgTypeDef);
IOPoolDeclare(AMUDFRRxIOPool, CanRxMsgTypeDef);
IOPoolDeclare(AMUDBLRxIOPool, CanRxMsgTypeDef);
IOPoolDeclare(AMUDBRRxIOPool, CanRxMsgTypeDef);
IOPoolDeclare(AMPLATERxIOPool, CanRxMsgTypeDef);
IOPoolDeclare(AMGETBULLETRxIOPool, CanRxMsgTypeDef);
//TxIOPool
IOPoolDeclare(CMTxIOPool, CanTxMsgTypeDef);
IOPoolDeclare(GMTxIOPool, CanTxMsgTypeDef);
IOPoolDeclare(AM1TxIOPool, CanTxMsgTypeDef);
IOPoolDeclare(AM2TxIOPool, CanTxMsgTypeDef);

#endif
