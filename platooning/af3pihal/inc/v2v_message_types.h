/** @file v2v_message_types.h
 * \brief Message types and type info for vehicle 2 vehicle communication.
 *
 * Created on: 12.05.2018
 * Author: Constantin
 */

#ifndef V2V_MESSAGE_TYPES_H_
#define V2V_MESSAGE_TYPES_H_

//type identifier 0x00 is RFU
#define V2V_MSG_TYPE_PLATOON 0x01  //!< type identifier for platoon Message
#define V2V_MSG_TYPE_LEADER_HEARTBEAT 0x02  /*!< type identifier for leader
											heartbeat */
#define V2V_MSG_TYPE_FOLLOWER 0x03  //!< type identifier for follower
#define V2V_MSG_TYPE_LEAVE_PLATOON 0x04  //!< type identifier for leave platoon
#define V2V_MSG_TYPE_SPLIT 0x05  //!< type identifier for split
#define V2V_MSG_TYPE_NEW_LEADER 0x06  //!< type identifier for new leader
#define V2V_MSG_TYPE_FUSE_PLATOON 0x07  //!< type identifier for fuse platoon
#define V2V_MSG_TYPE_HANDSHAKE 0x08 //!< type identifier for handshake

#endif /* V2V_MESSAGE_TYPES_H_ */
