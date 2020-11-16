/*******************************************************************************
 Copyright (C) 2010  Bryan Godbolt godbolt ( a t ) ualberta.ca
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
 ****************************************************************************/
/*
 This program sends some data to qgroundcontrol using the mavlink protocol.  The sent packets
 cause qgroundcontrol to respond with heartbeats.  Any settings or custom commands sent from
 qgroundcontrol are printed by this program along with the heartbeats.
 
 
 I compiled this program sucessfully on Ubuntu 10.04 with the following command
 
 gcc -I ../../pixhawk/mavlink/include -o udp-server udp-server-test.c
 
 the rt library is needed for the clock_gettime on linux
 */
/* These headers are for QNX, but should all be standard on unix/linux */
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h> /* required for the definition of bool in C99 */
#endif

#include "mavlink_udp.h"

struct sockaddr_in gcAddr; 
struct sockaddr_in locAddr;
int sock = 0;
uint8_t systemId = SYSTEM_ID;

int main(int argc, char* argv[])
{
	char help[] = "--help";	
	char target_ip[100];	
	float position[6] = {10.1, 23.7, 34.5, 3.2, 1.1, 3.3};
	uint8_t buf[BUFFER_LENGTH];
	ssize_t recsize;
	socklen_t fromlen = sizeof(gcAddr);
	int bytes_sent;
	mavlink_message_t msg;
	uint16_t len;
	int i = 0;

	strcpy(target_ip, CLIENT_IP);

	// Check if --help flag was used
	if(argc >= 2)
    {
    	if(strcmp(argv[1], help) == 0)
    	{
			printf("\n");
			printf("\tUsage:\n\n");
			printf("\t");
			printf("%s", argv[0]);
			printf(" <ip address of QGroundControl>\n");
			printf("\tDefault for localhost: udp-server 127.0.0.1\n\n");
			exit(EXIT_FAILURE);
		}
		else if(argc >=3 && (strcmp(argv[1], "sysid") == 0))
		{
			systemId = atoi(argv[2]);
		}
		else if(argc >=3 && (strcmp(argv[1], "ip") == 0))	// Change the target ip if parameter was given
		{
			strcpy(target_ip, argv[1]);
		}
    }
	


		// Socket Initial
	if((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
	{
		perror("[ERR] socket initial failed");
		exit(EXIT_FAILURE);
	}

	/* Attempt to make it non blocking */
#if (defined __QNX__) | (defined __QNXNTO__)
	if(fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) == -1)
#else
	if(fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC) == -1)
#endif
	{
		fprintf(stderr, "[ERR] setting nonblocking: %s\n", strerror(errno));
		close(sock);
		exit(EXIT_FAILURE);
	}
	
	// Server Address config
	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;			// AF_UNIX/AF_LOCAL/AF_INET/AF_INET6/PF_INET
	locAddr.sin_addr.s_addr = INADDR_ANY;	// inet_addr("127.0.0.1")
	locAddr.sin_port = htons(SERVER_PORT);
	
	/* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */ 
	if(bind(sock, (struct sockaddr *)&locAddr, sizeof(struct sockaddr)) == -1)
	{
		perror("[ERR] bind failed");
		close(sock);
		exit(EXIT_FAILURE);
	} 
	
	// Client Address config
	memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcAddr.sin_port = htons(CLIENT_PORT);
	
	printf("Start sending/receiving MAVLink message to/from QGroundControl...\n");

	while(1) 
    {
		// Send Messages

		/* Send Heartbeat HEARTBEAT */
		mavlink_msg_heartbeat_pack(systemId, COMPONENT_ID, &msg, 
									MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr));
		
		/* Send Status SYS_STATUS */
		mavlink_msg_sys_status_pack(systemId, COMPONENT_ID, &msg, 
									0, 0, 0, 500, 14800, 370, 80, 0, 0, 0, 0, 0, 0);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr));

	#if 0
		/* Send Local Position LOCAL_POSITION_NED */
		mavlink_msg_local_position_ned_pack(systemId, COMPONENT_ID, &msg, 
										microsSinceEpoch(), 
										position[0], position[1], position[2],
										position[3], position[4], position[5]);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr));
	#endif

		/* Send GLOBAL_POSITION_INT */
		mavlink_msg_global_position_int_pack(systemId, COMPONENT_ID, &msg, 
										microsSinceEpoch(), 
										GET_GPS_SCALED_VALUE(VEHICLE_GPS_LATITUDE), GET_GPS_SCALED_VALUE(VEHICLE_GPS_LONGITUDE), 
										VEHICLE_ALTITUDE_MSL, VEHICLE_ALTITUDE_GND, 
										0, 0, 0, UINT16_MAX);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr));
		
		/* Send attitude */
		mavlink_msg_attitude_pack(systemId, COMPONENT_ID, &msg, 
									microsSinceEpoch(), 0.785, 0.17444, 1.58, // 45deg, 10deg ,90deg
									0.01, 0.02, 0.03);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr));

		memset(buf, 0, BUFFER_LENGTH);

		// Receive Messages
		
		recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);

		if (recsize > 0)
      	{
			// Something received - print out all bytes and parse packet
			mavlink_message_t msg;
			mavlink_status_t status;
			
			//printf("Bytes Received: %d\nDatagram: ", (int)recsize);
			for (i = 0; i < recsize; i++)
			{
				//printf("%02x ", (unsigned char)buf[i]);
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status) == MAVLINK_FRAMING_OK)
				{
					// Decode Message
					AP_Decode_Message(&msg);
				}
			}
			//printf("\n");
		}
		memset(buf, 0, BUFFER_LENGTH);
		sleep(0.1); // Sleep one second
    }
}

/* QNX timer version */
#if (defined __QNX__) | (defined __QNXNTO__)
uint64_t microsSinceEpoch()
{
	struct timespec time;
	uint64_t micros = 0;
	
	clock_gettime(CLOCK_REALTIME, &time);  
	micros = (uint64_t)time.tv_sec * 1000000 + time.tv_nsec/1000;
	
	return micros;
}
#else
uint64_t microsSinceEpoch()
{
	struct timeval tv;
	uint64_t micros = 0;
	
	gettimeofday(&tv, NULL);  
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	
	return micros;
}
#endif

void AP_Decode_Message(mavlink_message_t* msg)
{
	switch(msg->msgid)
	{
		case MAVLINK_MSG_ID_HEARTBEAT:
		{ 
			mavlink_heartbeat_t payload;
			
	     	mavlink_msg_heartbeat_decode(msg, &payload); 

			printf("HEARTBEAT: TYPE:%d AUTOPILOT:%d MODE:%d SUB_MODE:%d STATUS:%d VER:0x%X\n", 
						payload.type, payload.autopilot, payload.base_mode,
						payload.custom_mode, payload.system_status,
						payload.mavlink_version);

			break; 
		}
		case MAVLINK_MSG_ID_SYSTEM_TIME:
		{ 
			mavlink_system_time_t payload;
			
	     	mavlink_msg_system_time_decode(msg, &payload); 

			printf("SYSTEM_TIME: UNIX_T:%luus TIME:%ums\n", 
						payload.time_unix_usec, payload.time_boot_ms);

			break; 
		}
		case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
		{ 
			mavlink_param_request_read_t payload;
			
	     	mavlink_msg_param_request_read_decode(msg, &payload); 

			printf("PARAM_REQUEST_READ: PARAM:%d NAME:%s SYS:%d COMP:%d\n", 
						payload.param_index, payload.param_id, payload.target_system, payload.target_component);

			AP_Send_Message_Param_Value(payload.param_index);
			
			break; 
		} 
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
		{ 
			mavlink_param_request_list_t payload;
			
	     	mavlink_msg_param_request_list_decode(msg, &payload); 

			printf("PARAM_REQUEST_LIST: SYS:%d COMP:%d\n", 
						payload.target_system, payload.target_component);

			AP_Send_Message_Param_Value(AP_PARAM_INDEX_TOTAL);
			
			break; 
		} 
		case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:	// Respond MISSION_COUNT
		{ 
			mavlink_mission_request_list_t payload;
			
	     	mavlink_msg_mission_request_list_decode(msg, &payload); 

			printf("MISSION_REQUEST_LIST: MISSION_TYPE:%d, SYS:%d COMP:%d\n", 
						payload.mission_type, payload.target_system, payload.target_system);

			AP_Send_Message_Mission_count(payload.mission_type);
			
			break; 
		} 
		case MAVLINK_MSG_ID_COMMAND_LONG:
		{ 
			mavlink_command_long_t payload;
			
	     	mavlink_msg_command_long_decode(msg, &payload); 

			printf("COMMAND_LONG: CMD:%d P:%f %f %f %f %f %f %f SYS:%d COMP:%d COMF:%d\n", 
						payload.command, 
						payload.param1, payload.param2, payload.param3, payload.param4,
						payload.param5, payload.param6, payload.param7,
						payload.target_system, payload.target_component, payload.confirmation);

			AP_Decode_Command_Long(&payload);

			break; 
		} 
		default:
			printf("\nReceived: MSG:%d SEQ:%d SYS:%d COMP:%d LEN:%d \n", 
					msg->msgid, msg->seq, msg->sysid, msg->compid, msg->len);
			break;
	}
}

void AP_Decode_Command_Long(mavlink_command_long_t* cmd)
{
	mavlink_message_t msg;
	uint8_t buf[BUFFER_LENGTH];
	uint16_t len;
	int bytes_sent;
	
	switch(cmd->command)
	{
		case MAV_CMD_REQUEST_PROTOCOL_VERSION:	// Respond PROTOCOL_VERSION
			if(cmd->param1)
			{
				mavlink_msg_protocol_version_pack(systemId, COMPONENT_ID, &msg, 
													200, 100, 200, NULL, NULL);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr));
			}
			break;
		case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:	// Respond AUTOPILOT_VERSION
			if(cmd->param1)
			{
				mavlink_msg_autopilot_version_pack(systemId, COMPONENT_ID, &msg, 
													AP_MAV_PROTOCOL_CAPABILITY, 6, 3, 0, 0, NULL, NULL, NULL, 2019, 1120, 0, NULL);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr));
			}
			break;
		default:
			break;
	}
}

void AP_Send_Message_Param_Value(uint16_t param_index)
{
	mavlink_message_t msg;
	uint8_t buf[BUFFER_LENGTH];
	uint16_t len;
	int bytes_sent;

	if(param_index == AP_PARAM_INDEX_TOTAL)
	{
		mavlink_msg_param_value_pack(systemId, COMPONENT_ID, &msg, 
									PARAM_NAME_ALTITUDE, 100, MAV_PARAM_TYPE_UINT8, AP_PARAM_INDEX_TOTAL, 0);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr));
	}
	else
	{
		mavlink_msg_param_value_pack(systemId, COMPONENT_ID, &msg, 
									PARAM_NAME_ALTITUDE, 100, MAV_PARAM_TYPE_UINT8, AP_PARAM_INDEX_TOTAL, 0);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr));
	}
}

void AP_Send_Message_Mission_count(uint8_t mission_type)
{
	mavlink_message_t msg;
	uint8_t buf[BUFFER_LENGTH];
	uint16_t len;
	int bytes_sent;

	switch(mission_type)
	{
		default:
		case MAV_MISSION_TYPE_MISSION:
			mavlink_msg_mission_count_pack(systemId, COMPONENT_ID, &msg, 
												systemId, COMPONENT_ID, 0, mission_type);
			break;
	}
	
	len = mavlink_msg_to_send_buffer(buf, &msg);
	bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr));
}


