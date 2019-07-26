/** @file v2v_message_types.h
 * \brief Message types and type info for vehicle 2 vehicle communication.
 *
 * Created on: 12.05.2018
 * Author: Constantin
 */

#ifndef V2V_MESSAGE_TYPES_H_
#define V2V_MESSAGE_TYPES_H_

//#include <stdint.h>

#define V2V_MSG_TYPE_DELETEME 0x00  //!< type identifier for DELETEME
#define V2V_MSG_TYPE_PLATOON 0x01  //!< type identifier for platoon Message
#define V2V_MSG_TYPE_LEADER_HEARTBEAT 0x02  /*!< type identifier for leader
											heartbeat */
#define V2V_MSG_TYPE_FOLLOWER 0x03  //!< type identifier for follower
#define V2V_MSG_TYPE_LEAVE_PLATOON 0x04  //!< type identifier for leave platoon
#define V2V_MSG_TYPE_SPLIT 0x05  //!< type identifier for split
#define V2V_MSG_TYPE_NEW_LEADER 0x06  //!< type identifier for new leader
#define V2V_MSG_TYPE_FUSE_PLATOON 0x07  //!< type identifier for fuse platoon
#define V2V_MSG_TYPE_HANDSHAKE 0x08 //!< type identifier for handshake

///** Template msg type - just a spacer*/
//typedef struct v2v_msg_deleteme_t {
//	uint8_t first_spacer_field;
//	uint16_t second_spacer_field;
//	uint32_t third_spacer_field;
//}__attribute__ ((packed)) v2v_msg_deleteme_t;  //!< the DELETEME message type
//#define V2V_MSG_SIZE_DELETEME sizeof(v2v_msg_deleteme_t)  //!< DELETEME msg len


///** Platoon msg type */
//typedef struct v2v_msg_platoon_t {
//	int platoon_id;
//	int leader_id;
//	int tail_id;
//}__attribute__ ((packed)) v2v_msg_platoon_t;  //!< the platoon message type
//#define V2V_MSG_SIZE_PLATOON sizeof(v2v_msg_platoon_t)  //!< platoon msg len


///** Leader heartbeat msg type*/
//typedef struct v2v_msg_leader_heartbeat_t {
//	uint8_t is_leader;
//	uint16_t vehicle_id;
//	uint8_t number_participants;
//	uint16_t list_of_participant_ids[UINT8_MAX];
//	uint8_t brake_status;
//}__attribute__ ((packed)) v2v_msg_leader_heartbeat_t;  /*!< the leader heartbeat
//														message type */
//#define V2V_MSG_SIZE_LEADER_HEARTBEAT sizeof(v2v_msg_leader_heartbeat_t)  /*!< leader
//																	heartbeat
//																	msg len */
///** Follower msg type*/
//typedef struct v2v_msg_follower_t {
//	uint16_t vehicle_id;
//	double heading_angle;
//	double steering_angle;
//	double vehicle_speed;
//	double vehicle_longitudinal_acc;
//	double acc_pedal_position;
//	double jaw_rate;
//	uint8_t brake_status;
//	uint8_t flags; //TODO check whether 8bit are enough
//}__attribute__ ((packed)) v2v_msg_follower_t;  //!< the follower message type
//#define V2V_MSG_SIZE_FOLLOWER sizeof(v2v_msg_follower_t)  //!< follower msg len


///** Leave platoon msg type */
//typedef struct v2v_msg_leave_paltoon_t {
//	uint16_t vehicle_id;
//}__attribute__ ((packed)) v2v_msg_leave_platoon_t;  /*!< the leave paltoon
//													message type */
//#define V2V_MSG_SIZE_LEAVE_PLATOON sizeof(v2v_msg_leave_platoon_t)  /*!< leave
//																	platoon msg
//																	len */
///** Split msg type */
//typedef struct v2v_msg_split_t {
//	uint16_t new_leader_id;
//}__attribute__ ((packed)) v2v_msg_split_t;  //!< the split message type
//#define V2V_MSG_SIZE_SPLIT sizeof(v2v_msg_split_t)  //!< split msg len


///** New leader msg type*/
//typedef struct v2v_msg_new_leader_t {
//	uint16_t new_leader_id;
//	uint16_t sender_id;
//	uint16_t old_leader_id;
//}__attribute__ ((packed)) v2v_msg_new_leader_t;  /*!< the new leader message
//												type */
//#define V2V_MSG_SIZE_NEW_LEADER sizeof(v2v_msg_new_leader_t)  /*!< new leader
//																msg len */

///** Fuse platoon msg type */
//typedef struct v2v_msg_fuse_platoon_t {
//	uint16_t sender_leader_id;
//	uint16_t target_leader_id;
//	uint8_t number_sender_participants;
//	uint16_t list_of_sender_participant_ids[UINT8_MAX];
//}__attribute__ ((packed)) v2v_msg_fuse_platoon_t;  /*!< the fuse platoon message
//													type */
//#define V2V_MSG_SIZE_FUSE_PLATOON sizeof(v2v_msg_fuse_platoon_t)  /*!< fuse
//																	platoon msg
//																	len */
#endif /* V2V_MESSAGE_TYPES_H_ */
