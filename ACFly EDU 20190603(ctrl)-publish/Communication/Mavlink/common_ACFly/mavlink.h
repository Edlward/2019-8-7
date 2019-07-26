/** @file
 *  @brief MAVLink comm protocol built from common_ACFly.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_H
#define MAVLINK_H

#define MAVLINK_PRIMARY_XML_IDX 0

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#ifndef MAVLINK_ENDIAN
#define MAVLINK_ENDIAN MAVLINK_LITTLE_ENDIAN
#endif

#ifndef MAVLINK_ALIGNED_FIELDS
#define MAVLINK_ALIGNED_FIELDS 1
#endif

#ifndef MAVLINK_CRC_EXTRA
#define MAVLINK_CRC_EXTRA 1
#endif

#ifndef MAVLINK_COMMAND_24BIT
#define MAVLINK_COMMAND_24BIT 1
#endif


#ifndef MAVLINK_EXTERNAL_RX_STATUS
#define MAVLINK_EXTERNAL_RX_STATUS
#endif

#ifndef MAVLINK_EXTERNAL_RX_BUFFER
#define MAVLINK_EXTERNAL_RX_BUFFER
#endif

#include "version.h"
#include "common_ACFly.h"

#endif // MAVLINK_H
