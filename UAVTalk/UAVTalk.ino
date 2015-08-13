/**
 ******************************************************************************
 *
 * @file       UAVTalk.ino
 * @author     Joerg-D. Rothfuchs
 * @brief      Implements a subset of the telemetry communication between
 * 	       OpenPilot CC, CC3D, Revolution and Ardupilot Mega MinimOSD
 * 	       with code from OpenPilot and MinimOSD.
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/> or write to the 
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include "UAVTalk.h"
#include "OSD_Vars.h"

#include <Adafruit_NeoPixel.h>

#define PIN 4
#define PIXEL 16;

#ifdef PROTOCOL_UAVTALK

Adafruit_NeoPixel ring = Adafruit_NeoPixel(16, PIN, NEO_GRB + NEO_KHZ800);


//#define DEBUG


static unsigned long last_gcstelemetrystats_send = 0;
static unsigned long last_flighttelemetry_connect = 0;
static uint8_t gcstelemetrystatus = TELEMETRYSTATS_STATE_DISCONNECTED;

#if defined VERSION_RELEASE_12_10_1 || defined VERSION_RELEASE_12_10_2 || defined VERSION_RELEASE_13_06_1 || defined VERSION_RELEASE_13_06_2
static uint32_t gcstelemetrystats_objid = GCSTELEMETRYSTATS_OBJID;
static uint8_t gcstelemetrystats_obj_len = GCSTELEMETRYSTATS_OBJ_LEN;
static uint8_t gcstelemetrystats_obj_status = GCSTELEMETRYSTATS_OBJ_STATUS;
static uint8_t flighttelemetrystats_obj_status = FLIGHTTELEMETRYSTATS_OBJ_STATUS;
#else
static uint32_t gcstelemetrystats_objid = GCSTELEMETRYSTATS_OBJID_001;
static uint8_t gcstelemetrystats_obj_len = GCSTELEMETRYSTATS_OBJ_LEN_001;
static uint8_t gcstelemetrystats_obj_status = GCSTELEMETRYSTATS_OBJ_STATUS_001;
static uint8_t flighttelemetrystats_obj_status = FLIGHTTELEMETRYSTATS_OBJ_STATUS_001;
#endif



// CRC lookup table
static const uint8_t crc_table[256] = {
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
	0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
	0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
	0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
	0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
	0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
	0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
	0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
	0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
	0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
	0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
	0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
	0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
	0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
	0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
	0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};


#ifdef DEBUG
void uavtalk_show_msg(uint8_t y, uavtalk_message_t *msg) {
	uint8_t *d;
	uint8_t i;
	uint8_t c;
	uint8_t crc;

	osd.setPanel(1, y);
	osd.openPanel();
	
	osd.printf("%6u header ", millis());
	c = (uint8_t) (msg->Sync);
	osd.printf("%2x ", c);
	crc = crc_table[0 ^ c];
	c = (uint8_t) (msg->MsgType);
	osd.printf("%2x ", c);
	crc = crc_table[crc ^ c];
	c = (uint8_t) (msg->Length & 0xff);
	osd.printf("%2x ", c);
	crc = crc_table[crc ^ c];
	c = (uint8_t) ((msg->Length >> 8) & 0xff);
	osd.printf("%2x ", c);
	crc = crc_table[crc ^ c];
	c = (uint8_t) (msg->ObjID & 0xff);
	osd.printf("%2x ", c);
	crc = crc_table[crc ^ c];
	c = (uint8_t) ((msg->ObjID >> 8) & 0xff);
	osd.printf("%2x ", c);
	crc = crc_table[crc ^ c];
	c = (uint8_t) ((msg->ObjID >> 16) & 0xff);
	osd.printf("%2x ", c);
	crc = crc_table[crc ^ c];
	c = (uint8_t) ((msg->ObjID >> 24) & 0xff);
	osd.printf("%2x ", c);
	crc = crc_table[crc ^ c];

#if defined VERSION_RELEASE_12_10_1 || defined VERSION_RELEASE_12_10_2 || defined VERSION_RELEASE_13_06_1 || defined VERSION_RELEASE_13_06_2
#else
	c = (uint8_t) (msg->InstID & 0xff);
	osd.printf("%2x ", c);
	crc = crc_table[crc ^ c];
	c = (uint8_t) ((msg->InstID >> 8) & 0xff);
	osd.printf("%2x ", c);
	crc = crc_table[crc ^ c];
#endif
	
	osd.printf("data ");
	if (msg->Length > HEADER_LEN) {
	  d = msg->Data;
	  for (i=0; i<msg->Length-HEADER_LEN; i++) {
		c = *d++;
	        osd.printf("%2x ", c);
		crc = crc_table[crc ^ c];
          }
	}
	
	osd.printf("crc ");
	osd.printf("%2x(%2x)", msg->Crc, crc);

	osd.closePanel();
}
#endif


static inline int8_t uavtalk_get_int8(uavtalk_message_t *msg, int pos) {
	return msg->Data[pos];
}


static inline int16_t uavtalk_get_int16(uavtalk_message_t *msg, int pos) {
	int16_t i;
	memcpy(&i, msg->Data+pos, sizeof(int16_t));
	return i;
}


static inline int32_t uavtalk_get_int32(uavtalk_message_t *msg, int pos) {
	int32_t i;
	memcpy(&i, msg->Data+pos, sizeof(int32_t));
	return i;
}


static inline float uavtalk_get_float(uavtalk_message_t *msg, int pos) {
	float f;
	memcpy(&f, msg->Data+pos, sizeof(float));
	return f;
}


void uavtalk_send_msg(uavtalk_message_t *msg) {
	uint8_t *d;
	uint8_t i;
	uint8_t c;
	
	if (op_uavtalk_mode & UAVTALK_MODE_PASSIVE)
		return;
	
	c = (uint8_t) (msg->Sync);
	Serial.write(c);
	msg->Crc = crc_table[0 ^ c];
	c = (uint8_t) (msg->MsgType);
	Serial.write(c);
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (uint8_t) (msg->Length & 0xff);
	Serial.write(c);
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (uint8_t) ((msg->Length >> 8) & 0xff);
	Serial.write(c);
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (uint8_t) (msg->ObjID & 0xff);
	Serial.write(c);
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (uint8_t) ((msg->ObjID >> 8) & 0xff);
	Serial.write(c);
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (uint8_t) ((msg->ObjID >> 16) & 0xff);
	Serial.write(c);
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (uint8_t) ((msg->ObjID >> 24) & 0xff);
	Serial.write(c);
	msg->Crc = crc_table[msg->Crc ^ c];

#if defined VERSION_RELEASE_12_10_1 || defined VERSION_RELEASE_12_10_2 || defined VERSION_RELEASE_13_06_1 || defined VERSION_RELEASE_13_06_2
#else
	c = 0; //(uint8_t) (msg->InstID & 0xff);
	Serial.write(c);
	msg->Crc = crc_table[msg->Crc ^ c];
	c = 0; //(uint8_t) ((msg->InstID >> 8) & 0xff);
	Serial.write(c);
	msg->Crc = crc_table[msg->Crc ^ c];
#endif
        
	if (msg->Length > HEADER_LEN) {
	  d = msg->Data;
	  for (i=0; i<msg->Length-HEADER_LEN; i++) {
		c = *d++;
		Serial.write(c);
		msg->Crc = crc_table[msg->Crc ^ c];
          }
	}
	Serial.write(msg->Crc);
}


void uavtalk_respond_object(uavtalk_message_t *msg_to_respond, uint8_t type) {
	uavtalk_message_t msg;
	
	msg.Sync	= UAVTALK_SYNC_VAL;
	msg.MsgType	= type;
	msg.Length	= RESPOND_OBJ_LEN;
	msg.ObjID	= msg_to_respond->ObjID;
	
	uavtalk_send_msg(&msg);
}


#if 0 // currently unused
void uavtalk_request_object(uint8_t id) {
	uavtalk_message_t msg;
	
	msg.Sync	= UAVTALK_SYNC_VAL;
	msg.MsgType	= UAVTALK_TYPE_OBJ_REQ;
	msg.Length	= REQUEST_OBJ_LEN;
	msg.ObjID	= id;
	
	uavtalk_send_msg(&msg);
}
#endif


void uavtalk_send_gcstelemetrystats(void) {
	uint8_t *d;
	uint8_t i;
	uavtalk_message_t msg;

	msg.Sync	= UAVTALK_SYNC_VAL;
	msg.MsgType	= UAVTALK_TYPE_OBJ_ACK;
	msg.Length	= gcstelemetrystats_obj_len + HEADER_LEN;
	msg.ObjID	= gcstelemetrystats_objid;

	d = msg.Data;
	for (i=0; i<gcstelemetrystats_obj_len; i++) {
		*d++ = 0;
	}

	msg.Data[gcstelemetrystats_obj_status] = gcstelemetrystatus;
	// remaining data unused and unset
	
	uavtalk_send_msg(&msg);
	last_gcstelemetrystats_send = millis();
}


uint8_t uavtalk_parse_char(uint8_t c, uavtalk_message_t *msg) {
	static uint8_t status = UAVTALK_PARSE_STATE_WAIT_SYNC;
	static uint8_t crc = 0;
	static uint8_t cnt = 0;

	switch (status) {
		case UAVTALK_PARSE_STATE_WAIT_SYNC:
			if (c == UAVTALK_SYNC_VAL) {
				status = UAVTALK_PARSE_STATE_GOT_SYNC;
				msg->Sync = c;
				crc = crc_table[0 ^ c];
			}
		break;
		case UAVTALK_PARSE_STATE_GOT_SYNC:
			crc = crc_table[crc ^ c];
			if ((c & UAVTALK_TYPE_MASK) == UAVTALK_TYPE_VER) {
				status = UAVTALK_PARSE_STATE_GOT_MSG_TYPE;
				msg->MsgType = c;
				cnt = 0;
			}
			else {
				status = UAVTALK_PARSE_STATE_WAIT_SYNC;
			}
		break;
		case UAVTALK_PARSE_STATE_GOT_MSG_TYPE:
			crc = crc_table[crc ^ c];
			cnt++;
			if (cnt < 2) {
				msg->Length = ((uint16_t) c);
			}
			else {
				msg->Length += ((uint16_t) c) << 8;
                                if ((msg->Length < HEADER_LEN) || (msg->Length > 255 + HEADER_LEN)) {
                                       // Drop corrupted messages:
                                       // Minimal length is HEADER_LEN
                                       // Maximum is HEADER_LEN + 255 (Data) + 2 (Optional Instance Id)
                                       // As we are not parsing Instance Id, 255 is a hard maximum. 
				       status = UAVTALK_PARSE_STATE_WAIT_SYNC;
                                } else {
				       status = UAVTALK_PARSE_STATE_GOT_LENGTH;
				       cnt = 0;
                                }
			}
		break;
		case UAVTALK_PARSE_STATE_GOT_LENGTH:
			crc = crc_table[crc ^ c];
			cnt++;
			switch (cnt) {
				case 1:
					msg->ObjID = ((uint32_t) c);
				break;
				case 2:
					msg->ObjID += ((uint32_t) c) << 8;
				break;
				case 3:
					msg->ObjID += ((uint32_t) c) << 16;
				break;
				case 4:
					msg->ObjID += ((uint32_t) c) << 24;
#if defined VERSION_RELEASE_12_10_1 || defined VERSION_RELEASE_12_10_2 || defined VERSION_RELEASE_13_06_1 || defined VERSION_RELEASE_13_06_2
					if (msg->Length == HEADER_LEN) { // no data exists
						status = UAVTALK_PARSE_STATE_GOT_DATA;
					} else {
						status = UAVTALK_PARSE_STATE_GOT_INSTID;
                                        }
#else
					status = UAVTALK_PARSE_STATE_GOT_OBJID;
#endif
					cnt = 0;
				break;
			}
		break;
		case UAVTALK_PARSE_STATE_GOT_OBJID:
			crc = crc_table[crc ^ c];
			cnt++;
			switch (cnt) {
				case 1:
					msg->InstID = ((uint32_t) c);
				break;
				case 2:
					msg->InstID += ((uint32_t) c) << 8;
					if (msg->Length == HEADER_LEN) { // no data exists
						status = UAVTALK_PARSE_STATE_GOT_DATA;
					} else {
						status = UAVTALK_PARSE_STATE_GOT_INSTID;
					}
					cnt = 0;
				break;
			}
		break;
		case UAVTALK_PARSE_STATE_GOT_INSTID:
			crc = crc_table[crc ^ c];
			cnt++;
			msg->Data[cnt - 1] = c;
			if (cnt >= msg->Length - HEADER_LEN) {
				status = UAVTALK_PARSE_STATE_GOT_DATA;
				cnt = 0;
			}
		break;
		case UAVTALK_PARSE_STATE_GOT_DATA:
			msg->Crc = c;
			status = UAVTALK_PARSE_STATE_GOT_CRC;
		break;
	}
	
	if (status == UAVTALK_PARSE_STATE_GOT_CRC) {
		status = UAVTALK_PARSE_STATE_WAIT_SYNC;
		if (crc == msg->Crc) {
			return msg->Length;
		} else {
			return 0;
		}
	} else {
		return 0;
	}
}


int uavtalk_read(void) {
	static uint8_t crlf_count = 0;
	static uavtalk_message_t msg;
	uint8_t show_prio_info = 0;
	
	// grabbing data
	while (!show_prio_info && Serial.available() > 0) {
		uint8_t c = Serial.read();
		
//		// needed for MinimOSD upload, while no UAVTalk is established
//		if (gcstelemetrystatus == TELEMETRYSTATS_STATE_DISCONNECTED && millis() < 20000 && millis() > 5000) {
//			if (c == '\n' || c == '\r') {
//				crlf_count++;
//			} else {
//				crlf_count = 0;
//			}
//			if (crlf_count == 3) {
//				//uploadFont();
//			}
//		}
		
		// parse data to msg
		if (uavtalk_parse_char(c, &msg)) {
			// consume msg
			switch (msg.ObjID) {
				case FLIGHTTELEMETRYSTATS_OBJID:
#ifdef VERSION_ADDITIONAL_UAVOBJID
				case FLIGHTTELEMETRYSTATS_OBJID_001:
#endif
					switch (msg.Data[flighttelemetrystats_obj_status]) {
						case TELEMETRYSTATS_STATE_DISCONNECTED:
							gcstelemetrystatus = TELEMETRYSTATS_STATE_HANDSHAKEREQ;
							uavtalk_send_gcstelemetrystats();

						break;
						case TELEMETRYSTATS_STATE_HANDSHAKEACK:
							gcstelemetrystatus = TELEMETRYSTATS_STATE_CONNECTED;
							uavtalk_send_gcstelemetrystats();
						break;
						case TELEMETRYSTATS_STATE_CONNECTED:
							gcstelemetrystatus = TELEMETRYSTATS_STATE_CONNECTED;
							last_flighttelemetry_connect = millis();
						break;
					}
				break;
				case ATTITUDEACTUAL_OBJID:
				case ATTITUDESTATE_OBJID:
					last_flighttelemetry_connect = millis();
					show_prio_info = 1;
        				osd_roll		= (int16_t) uavtalk_get_float(&msg, ATTITUDEACTUAL_OBJ_ROLL);
        				osd_pitch		= (int16_t) uavtalk_get_float(&msg, ATTITUDEACTUAL_OBJ_PITCH);
        				osd_yaw			= (int16_t) uavtalk_get_float(&msg, ATTITUDEACTUAL_OBJ_YAW);
                                        // if we don't have a GPS, use Yaw for heading
                                        if (osd_lat == 0) {
                                            osd_heading = osd_yaw;
                                        }
				break;
				case FLIGHTSTATUS_OBJID:
#ifdef VERSION_ADDITIONAL_UAVOBJID
				case FLIGHTSTATUS_OBJID_001:
				case FLIGHTSTATUS_OBJID_002:
				case FLIGHTSTATUS_OBJID_003:
				case FLIGHTSTATUS_OBJID_004:
				case FLIGHTSTATUS_OBJID_005:
#endif
        				osd_armed		= uavtalk_get_int8(&msg, FLIGHTSTATUS_OBJ_ARMED);
        				osd_mode		= uavtalk_get_int8(&msg, FLIGHTSTATUS_OBJ_FLIGHTMODE);
				break;
				case MANUALCONTROLCOMMAND_OBJID:
#ifdef VERSION_ADDITIONAL_UAVOBJID
				case MANUALCONTROLCOMMAND_OBJID_001:
				case MANUALCONTROLCOMMAND_OBJID_002:
#endif
					osd_throttle		= (int16_t) (100.0 * uavtalk_get_float(&msg, MANUALCONTROLCOMMAND_OBJ_THROTTLE));
					if (osd_throttle < 0 || osd_throttle > 200) osd_throttle = 0;
					// Channel mapping:
					// 0   is Throttle
                                        // 1-2 are Roll / Pitch 
                                        // 3   is Yaw
                                        // 4   is Mode
                                        // 5   is Collective (Heli - constant 65536 otherwhise)
                                        // 6-8 are Accessory 0-2
                                        // In OPOSD:
                                        // chanx_raw     used for menu navigation (Roll/pitch)
                                        // osd_chanx_raw used for panel navigation (Accessory)
                                        chan1_raw		= uavtalk_get_int16(&msg, MANUALCONTROLCOMMAND_OBJ_CHANNEL_1);
					chan2_raw		= uavtalk_get_int16(&msg, MANUALCONTROLCOMMAND_OBJ_CHANNEL_2);
					osd_chan5_raw		= uavtalk_get_int16(&msg, MANUALCONTROLCOMMAND_OBJ_CHANNEL_4);
					osd_chan6_raw		= uavtalk_get_int16(&msg, MANUALCONTROLCOMMAND_OBJ_CHANNEL_6);
					osd_chan7_raw		= uavtalk_get_int16(&msg, MANUALCONTROLCOMMAND_OBJ_CHANNEL_7);
					osd_chan8_raw		= uavtalk_get_int16(&msg, MANUALCONTROLCOMMAND_OBJ_CHANNEL_8);
				break;
#ifndef GPS_SIMULATION
				case GPSPOSITION_OBJID:
				case GPSPOSITIONSENSOR_OBJID:
				case GPSPOSITIONSENSOR_OBJID_001:
					osd_lat			= uavtalk_get_int32(&msg, GPSPOSITION_OBJ_LAT) / 10000000.0;
					osd_lon			= uavtalk_get_int32(&msg, GPSPOSITION_OBJ_LON) / 10000000.0;
					osd_satellites_visible	= uavtalk_get_int8(&msg, GPSPOSITION_OBJ_SATELLITES);
					osd_fix_type		= uavtalk_get_int8(&msg, GPSPOSITION_OBJ_STATUS);
					osd_heading		= uavtalk_get_float(&msg, GPSPOSITION_OBJ_HEADING);
					osd_alt			= uavtalk_get_float(&msg, GPSPOSITION_OBJ_ALTITUDE);
					osd_groundspeed		= uavtalk_get_float(&msg, GPSPOSITION_OBJ_GROUNDSPEED);
				break;
#endif
// because of #define PIOS_GPS_MINIMAL in the OP flight code, the following is unfortunately currently not supported:
#if 0
				case GPSTIME_OBJID:
        				osd_time_hour		= uavtalk_get_int8(&msg, GPSTIME_OBJ_HOUR);
        				osd_time_minute		= uavtalk_get_int8(&msg, GPSTIME_OBJ_MINUTE);
				break;
#endif
				case GPSVELOCITY_OBJID:
				case GPSVELOCITYSENSOR_OBJID:
					osd_climb		= -1.0 * uavtalk_get_float(&msg, GPSVELOCITY_OBJ_DOWN);
				break;
#ifdef FLIGHT_BATT_ON_REVO
				case FLIGHTBATTERYSTATE_OBJID:
				case FLIGHTBATTERYSTATE_OBJID_001:
        				osd_vbat_A		= uavtalk_get_float(&msg, FLIGHTBATTERYSTATE_OBJ_VOLTAGE);
					osd_curr_A		= (int16_t) (100.0 * uavtalk_get_float(&msg, FLIGHTBATTERYSTATE_OBJ_CURRENT));
					osd_total_A		= (int16_t) uavtalk_get_float(&msg, FLIGHTBATTERYSTATE_OBJ_CONSUMED_ENERGY);
					osd_est_flight_time	= (int16_t) uavtalk_get_float(&msg, FLIGHTBATTERYSTATE_OBJ_ESTIMATED_FLIGHT_TIME);
				break;
#endif
#ifdef REVO_ADD_ONS
				case BAROALTITUDE_OBJID:
				case BAROSENSOR_OBJID:
					revo_baro_alt		= (int16_t) uavtalk_get_float(&msg, BAROALTITUDE_OBJ_ALTITUDE);
				break;
				case OPLINKSTATUS_OBJID:
#ifdef VERSION_ADDITIONAL_UAVOBJID
				case OPLINKSTATUS_OBJID_001:
				case OPLINKSTATUS_OBJID_002:
#endif
        				oplm_rssi		= uavtalk_get_int8(&msg, OPLINKSTATUS_OBJ_RSSI);
        				oplm_linkquality	= uavtalk_get_int8(&msg, OPLINKSTATUS_OBJ_LINKQUALITY);
				break;
#endif
#ifdef OP_DEBUG
				case SYSTEMALARMS_OBJID:
#ifdef VERSION_ADDITIONAL_UAVOBJID
				case SYSTEMALARMS_OBJID_001:
				case SYSTEMALARMS_OBJID_002:
				case SYSTEMALARMS_OBJID_003:
				case SYSTEMALARMS_OBJID_004:
				case SYSTEMALARMS_OBJID_005:
#endif
					op_alarm  = msg.Data[SYSTEMALARMS_ALARM_CPUOVERLOAD];
//					op_alarm += msg.Data[SYSTEMALARMS_ALARM_EVENTSYSTEM] * 0x10;
					op_alarm += msg.Data[SYSTEMALARMS_ALARM_MANUALCONTROL] * 0x10;
					if (op_alarm > 0x11) show_prio_info = 1;
				break;
#endif
				
				// TODO implement more X_OBJID for more OSD data 
				// osd_waypoint_seq = 0;           // waypoint sequence
				// osd_airspeed = 0;               // air speed (only with pitot tube)
				// etc.
			}
			if (msg.MsgType == UAVTALK_TYPE_OBJ_ACK) {
				uavtalk_respond_object(&msg, UAVTALK_TYPE_ACK);
			}
		}

		delayMicroseconds(190);  // wait at least 1 byte
	}
	
	// check connect timeout
	if (last_flighttelemetry_connect + FLIGHTTELEMETRYSTATS_CONNECT_TIMEOUT < millis()) {
		gcstelemetrystatus = TELEMETRYSTATS_STATE_DISCONNECTED;
		show_prio_info = 1;
	}
	
	// periodically send gcstelemetrystats
	if (last_gcstelemetrystats_send + GCSTELEMETRYSTATS_SEND_PERIOD < millis()) {
		uavtalk_send_gcstelemetrystats();
	}

        return show_prio_info;
}


int uavtalk_state(void)
{
	return gcstelemetrystatus;
}

#endif // PROTOCOL_UAVTALK

const uint32_t RED = ring.Color(255, 0, 0);
const uint32_t GREEN = ring.Color(0, 255, 0);
const uint32_t BLUE = ring.Color(0, 0, 255);
const uint32_t MAGENTA = ring.Color(255, 0, 255);
const uint32_t YELLOW = ring.Color(255, 255, 0);
const uint32_t PINK = ring.Color(219,112,147);
const uint32_t SKY_BLUE = ring.Color(135,206,255);
const uint32_t MAIZE = ring.Color(128,158,10);
const uint32_t LAVENDER = ring.Color(88,2,163);
const uint32_t SEA_FOAM = ring.Color(32,178,170);
const uint32_t SPleds = ring.Color(102,205,0);
const uint32_t DARK_ORANGE = ring.Color(237,180,6);
const uint32_t ORANGE = ring.Color(237,120,6);
const uint32_t WHITE = ring.Color(255,255,255);
const uint32_t BLACK = ring.Color(0,0,0);

void setup() {
  Serial.begin(57600);
  ring.begin();
  ring.show(); // Initialize all pixels to 'off'
  ring.setBrightness(100);
  ring.setPixelColor(15,0x202020);
}

void loop() {
   int returncode = uavtalk_read();

   // armed
   if (osd_armed > 0) { // does only work sometimes!
     ring.setPixelColor(0, RED); // armed = red
   } else {
     ring.setPixelColor(0, BLACK);
   }

   // yaw
   uint32_t value = (osd_yaw + 180) * 25 / 36; // 0 - 250
   if (value > 255) {
     value = 255;
   }
   uint32_t color =  value;
   ring.setPixelColor(1, color);
   
   // throttle (0-100)
   value = osd_throttle;
   value = value * 2.5;
   color = value << 16; // red
   ring.setPixelColor(2, color);

   // flight mode
   if (osd_mode == 1) {
     ring.setPixelColor(3, GREEN); // green
   } else if (osd_mode == 2) {
     ring.setPixelColor(3, YELLOW); // yellow
   } else if (osd_mode == 3) {
     ring.setPixelColor(3, RED); // red
   } else {
     ring.setPixelColor(3, LAVENDER); // 
   }

   ring.show();
}

