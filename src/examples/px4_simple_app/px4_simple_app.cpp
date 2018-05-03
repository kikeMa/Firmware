/****************************************************************************
*
*   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

/**
* @file px4_simple_app.c
* Minimal application example for PX4 autopilot
*
* @author Example User <mail@example.com>
*/

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <mavlink/mavlink_mission.h>
#include <mavlink/mavlink_main.h>
#include <mavlink/mavlink_command_sender.h>


//#include <mc_pos_control/mc_pos_control_main.cpp>
#include <uORB/topics/sensor_baro.h>	// Altura por barometro
#include <uORB/topics/vehicle_attitude.h>	// velocidad (rollspeed, pitchspeed, yawspeed)
#include <uORB/topics/distance_sensor.h>	// sensor de distancia (min_distance, max_distance, current_distance, coveriance ,MAV_DISTANCE_SENSOR_LASER)
#include <uORB/topics/vehicle_control_mode.h>	// sensor de distancia (min_distance, max_distance, current_distance, coveriance ,MAV_DISTANCE_SENSOR_LASER)
//#include <uORb/topics/vehicle_local_position.h>	// altura (z)

extern "C" __EXPORT int px4_simple_app_main(int argc, char *argv[]);

class SensorDistancia;

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

namespace sensor_distancia_q
{
	SensorDistancia *instance;
} // namespace attitude_estimator_q

class SensorDistancia
{
public:

	/**
	* Constructor
	*/
	SensorDistancia();

	/**
	* Destructor, also kills task.
	*/
	~SensorDistancia();


	int start();

	static void task_main_sensor_distancia(int argc, char *argv[]);

	int send_mission_count(struct sockaddr_in gcAddr, int sock);

	void task_main();
private:
	bool		_task_should_exit = false;		/**< if true, task should exit */
	int		_control_task = -1;			/**< task handle for task */

};

SensorDistancia::SensorDistancia()
{

}

SensorDistancia::~SensorDistancia()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	sensor_distancia_q::instance = nullptr;
}

int auxKike = 1;


int SensorDistancia::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("sensor_distancia",
	SCHED_DEFAULT,
	SCHED_PRIORITY_ESTIMATOR,
	2000,
	(px4_main_t)&SensorDistancia::task_main_sensor_distancia,
	nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void SensorDistancia::task_main_sensor_distancia(int argc, char *argv[])
{
	sensor_distancia_q::instance->task_main();
}


void SensorDistancia::task_main()
{
	PX4_INFO("Hello Sky!");

	int sensor_sub_fd_sensor = orb_subscribe(ORB_ID(distance_sensor));
	int vehicle_control_mode_fd = orb_subscribe(ORB_ID(vehicle_control_mode));
	int vehicle_status_fd = orb_subscribe(ORB_ID(vehicle_status));

	orb_set_interval(sensor_sub_fd_sensor, 200);
	orb_set_interval(vehicle_control_mode_fd, 200);
	orb_set_interval(vehicle_status_fd, 200);

	sleep(5); // Sleep one second

	char target_ip[100];

	struct sockaddr_in locAddr;
	//struct sockaddr_in fromAddr;
	uint8_t buf[BUFFER_LENGTH];
	ssize_t recsize;
	socklen_t fromlen;
	int i = 0;
	//int success = 0;
	unsigned int temp = 0;

	strcpy(target_ip, "127.0.0.1");
	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(14551);
	if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr))){
		PX4_INFO("Error Bind");
		return;
	}


	for (;;)
	{
		send_new_mission();
		memset(buf, 0, BUFFER_LENGTH);
		recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
		if (recsize > 0){
			// Something received - print out all bytes and parse packet

			mavlink_message_t msg;
			msg.sysid = 0;
			msg.compid = 0;
			msg.len = 0;
			msg.msgid = 0;

			mavlink_status_t status;

			printf("Bytes Received: %d\nDatagram: ", (int)recsize);

			for (i = 0; i < recsize; ++i)
			{
				temp = buf[i];
				printf("%02x ", (unsigned char)temp);
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
				{
					// Packet received
					printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
					if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST) {
						mavlink_mission_request_t mission_request;
						mavlink_msg_mission_request_decode(&msg, &mission_request);
						printf("\nmission request:, \n \tSEQ: %d, \n \tMISSION_TYPE: %d, \n", mission_request.seq,mission_request.mission_type);


					}
				}
			}
			printf("\n");
		}

		sleep(1); // Sleep one second
	}

	/*

	while(!_task_should_exit){
	struct distance_sensor_s distance_sensor_param;
	struct vehicle_control_mode_s vehicle_control_mode_param;
	struct vehicle_status_s status;

	orb_copy(ORB_ID(distance_sensor), sensor_sub_fd_sensor, &distance_sensor_param);
	orb_copy(ORB_ID(vehicle_control_mode), vehicle_control_mode_fd, &vehicle_control_mode_param);
	orb_copy(ORB_ID(vehicle_status), vehicle_status_fd, &status);



	if (0.5f < distance_sensor_param.current_distance){
	*//*
	PX4_INFO("distance_sensor:\n\tmin_distance\t%8.4f\n\tmax_distance\t%8.4f\n\tcurrent_distance\t%8.4f\n\tcovariance\t%8.4f\n\ttype\t%d\n\tOrientation\t%d\n\n",
	(double)distance_sensor_param.min_distance,
	(double)distance_sensor_param.max_distance,
	(double)distance_sensor_param.current_distance,
	(double)distance_sensor_param.covariance,
	(uint8_t)distance_sensor_param.type,
	(uint16_t)distance_sensor_param.orientation);

	PX4_INFO("vehicle_control_mode:\n\tflag_armed\t%d\n\tflag_control_auto_enabled\t%d\n\tflag_control_manual_enabled\t%d\n\n",
	vehicle_control_mode_param.flag_armed,
	vehicle_control_mode_param.flag_control_auto_enabled,
	vehicle_control_mode_param.flag_control_manual_enabled
);
*/
//	if ( distance_sensor_param.current_distance < 10 && auxKike == 1){
/*
PX4_INFO("vehicle_control_mode:\n\tflag_armed\t%d\n\tflag_control_auto_enabled\t%d\n\tflag_control_manual_enabled\t%d\n\n",
vehicle_control_mode_param.flag_armed,
vehicle_control_mode_param.flag_control_auto_enabled,
vehicle_control_mode_param.flag_control_manual_enabled
);
*/


/*
// Con este comando pausamos la misión y para que no se choque.

struct vehicle_command_s cmd = {
.timestamp = 0,
.param5 = NAN,
.param6 = NAN,
.param1 = 1,
.param2 = 4,
.param3 = 3,
.param4 = NAN,
.param7 = NAN,
.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE,
.target_system = status.system_id,
.target_component = status.component_id,
.source_system = 1,
.source_component = 2,
.confirmation = 1,
.from_external = true
};
orb_advert_t h = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
(void)orb_unadvertise(h);
*/
/*
const struct vehicle_command_s cmd {
.timestamp = 0,
.param1 = 0,
.param2 = 1,
.param3 = 0,
.param4 = NAN,
.param5 = 47.396675f,
.param6 = 8.550455f,
.param7 = 10.000000f,
.command = MAV_CMD_NAV_WAYPOINT,
.target_system = status.system_id,
.target_component = status.component_id,
.source_system = 1,
.source_component = 2,
.confirmation = 1,
.from_external = true
};

MavlinkCommandSender::instance().handle_vehicle_command(cmd, mavlink_channel_t(MAVLINK_COMM_0) );
*/
/*
const mavlink_mission_count_t wpc {
.count = 1,
.target_system = 1,
.target_component = 190,
.mission_type = 0
};

//Creo el mensaje con el waypoint
const mavlink_mission_item_t wp {
.param1 = 0.0f,
.param2 = 0.0f,
.param3 = 0.0f,
.param4 = NAN,
.x = 47.396675f,
.y = 8.550455f,
.z = 10.000000f,
.seq = 0,
.command = 22,
.target_system = 1,
.target_component = 190,
.frame = 3,
.current = 1,
.autocontinue = 1,
.mission_type = 0
};

// Imprimo mensaje creado anteriomente
PX4_INFO("El mensaje creado es: \n\t	param1 %f,\n\t	param2 %f,\n\t	param3 %f,\n\t	param4 %f,\n\t	x %f,\n\t	y %f,\n\t	z %f,\n\t	seq %d,\n\t	command %d,\n\t	target_system %d,\n\t	target_component %d,\n\t	frame %d,\n\t	current %d,\n\t	autocontinue %d,\n\t	mission_type %d,\n", wp.param1,
wp.param2,
wp.param3,
wp.param4,
wp.x,
wp.y,
wp.z,
wp.seq,
wp.command,
wp.target_system,
wp.target_component,
wp.frame,
wp.current,
wp.autocontinue,
wp.mission_type);

mavlink_message_t msgc;
mavlink_message_t msg;

PX4_INFO("El mensaje cr1");

Mavlink * mavlink = new Mavlink();

PX4_INFO("El mensaje cr2");

uint8_t system_id = mavlink->get_system_id();
uint8_t component_id = mavlink->get_component_id();

PX4_INFO("El mensaje cr3");

// Genero mensaje como si enviara desde mavlink
// uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_item_t* mission_item)
mavlink_msg_mission_item_encode(system_id, component_id, &msg, &wp);
mavlink_msg_mission_count_encode(255, 0, &msgc, &wpc);

PX4_INFO("El mensaje cr4");

// Llamo al manejador de mavlink_mission
MavlinkMissionManager *_mission_manager = new MavlinkMissionManager(mavlink);
PX4_INFO("El mensaje cr5");

_mission_manager->handle_message(&msgc);

PX4_INFO("El mensaje cr6");

//_mission_manager->handle_message(&msg);


PX4_INFO("El mensaje cr");
*/

/*
// Entro dentro de state posctl pero necesito
cmd = {
.timestamp = 0,
.param5 = NAN,
.param6 = NAN,
.param1 = 1,
.param2 = 3,
.param3 = 0,
.param4 = NAN,
.param7 = NAN,
.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE,
.target_system = status.system_id,
.target_component = status.component_id,
.source_system = 1,
.source_component = 2,
.confirmation = 1,
.from_external = true
};
*/
//	h = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
//	(void)orb_unadvertise(h);

/*
PX4_INFO("vehicle_control_mode:\n\tflag_armed\t%d\n\tflag_control_auto_enabled\t%d\n\tflag_control_manual_enabled\t%d\n\n",
vehicle_control_mode_param.flag_armed,
vehicle_control_mode_param.flag_control_auto_enabled,
auxKike
);
auxKike = 0;*/
/*
struct vehicle_command_s cmd = {
.timestamp = 0,
.param5 = NAN,
.param6 = NAN,
.param1 = NAN,
.param2 = NAN,
.param3 = NAN,
.param4 = NAN,
.param7 = NAN,
.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND,
.target_system = status.system_id,
.target_component = status.component_id,
.source_system = 1,
.source_component = 2,
.confirmation = 1,
.from_external = true
};

orb_advert_t h = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
(void)orb_unadvertise(h);

*//*
}
}

}*/
PX4_INFO("GoodBye");
}

int SensorDistancia::send_mission_count(struct sockaddr_in gcAddr, int sock){

	mavlink_message_t msg;
	uint16_t len;
	uint8_t buf[BUFFER_LENGTH];

	const mavlink_mission_count_t wpc {
		.count = 1,
		.target_system = 1,
		.target_component = 1,
		.mission_type = 0
	};

	// Generamos mensaje
	mavlink_msg_mission_count_encode(255, 0, &msg, &wpc);
	len = mavlink_msg_to_send_buffer(buf, &msg);

	uint16_t bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr_in));
	if (bytes_sent != len){
		PX4_WARN("Error al enviar count_mission");
		return -1;
	}
	return bytes_sent;
}

int SensorDistancia::send_new_mission(){

	int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	struct sockaddr_in gcAddr;

	char target_ip[10];
	strcpy(target_ip, "127.0.0.1");


	memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcAddr.sin_port = htons(14557);

	// Enviamos count a para establecer nueva misión
	int leng_send_count = send_mission_count(gcAddr, sock);
	PX4_INFO("LENG SEND COUNT %d", leng_send_count);


}

int px4_simple_app_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: attitude_estimator_q {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (sensor_distancia_q::instance != nullptr) {
			warnx("already running");
			return 1;
		}

		sensor_distancia_q::instance = new SensorDistancia;

		if (sensor_distancia_q::instance == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != sensor_distancia_q::instance->start()) {
			delete sensor_distancia_q::instance;
			sensor_distancia_q::instance = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (sensor_distancia_q::instance == nullptr) {
			warnx("not running");
			return 1;
		}

		delete sensor_distancia_q::instance;
		sensor_distancia_q::instance = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (sensor_distancia_q::instance) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
