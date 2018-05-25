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
#include <uORB/topics/position_setpoint_triplet.h>	// Altura por barometro
#include <uORB/topics/vehicle_attitude.h>	// velocidad (rollspeed, pitchspeed, yawspeed)
#include <uORB/topics/distance_sensor.h>	// sensor de distancia (min_distance, max_distance, current_distance, coveriance ,MAV_DISTANCE_SENSOR_LASER)
#include <uORB/topics/vehicle_control_mode.h>	// sensor de distancia (min_distance, max_distance, current_distance, coveriance ,MAV_DISTANCE_SENSOR_LASER)
#include <uORB/topics/vehicle_global_position.h>	// altura (z)

extern "C" __EXPORT int px4_simple_app_main(int argc, char *argv[]);

class SensorDistancia;


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

	int send_mission_count(uint16_t count, uint8_t target_system, uint8_t target_component, uint8_t mission_type);
	int connect_mavlink();
	int send_new_mission(mavlink_mission_count_t mission_count ,mavlink_mission_item_int_t * items , int32_t * newWaypoint, uint16_t pos);
	int send_request_list(uint8_t mission_type);
	int recibir_request_int();
	int send_item(float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z, uint16_t seq, uint16_t command, uint8_t target_system, uint8_t target_component, uint8_t frame, uint8_t current, uint8_t autocontinue, uint8_t mission_type);
	int recibir_ack();
	mavlink_mission_count_t receive_mission_count();
	int receive_mission(mavlink_mission_count_t * mission_count, mavlink_mission_item_int_t ** items);
	int send_request_int(uint16_t seq, uint8_t target_system, uint8_t target_component, uint8_t mission_type);
	int send_request(uint16_t seq, uint8_t target_system, uint8_t target_component, uint8_t mission_type);
	mavlink_mission_item_int_t receive_mission_item_int();
	int send_mission_ack(uint8_t target_system, uint8_t target_component, uint8_t  ,uint8_t mission_type);
	void obtain_new_point(struct position_setpoint_triplet_s position_setpoint_triplet, int32_t * newPoint, struct vehicle_global_position_s vehicle_global_position);

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

// Variables globales mavlink
struct sockaddr_in locAddr;
struct sockaddr_in gcAddr;
#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
int sock;


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
	int position_setpoint_triplet_fd = orb_subscribe(ORB_ID(position_setpoint_triplet));
	int vehicle_global_position_fd = orb_subscribe(ORB_ID(vehicle_global_position));

	orb_set_interval(sensor_sub_fd_sensor, 200);
	orb_set_interval(vehicle_control_mode_fd, 200);
	orb_set_interval(vehicle_status_fd, 200);
	orb_set_interval(position_setpoint_triplet_fd, 200);
	orb_set_interval(vehicle_global_position_fd, 200);


	while(!_task_should_exit){
		struct distance_sensor_s distance_sensor_param;
		struct vehicle_control_mode_s vehicle_control_mode_param;
		struct vehicle_status_s status;
		struct position_setpoint_triplet_s position_setpoint_triplet;
		struct vehicle_global_position_s vehicle_global_position;

		orb_copy(ORB_ID(distance_sensor), sensor_sub_fd_sensor, &distance_sensor_param);
		orb_copy(ORB_ID(vehicle_control_mode), vehicle_control_mode_fd, &vehicle_control_mode_param);
		orb_copy(ORB_ID(vehicle_status), vehicle_status_fd, &status);
		orb_copy(ORB_ID(position_setpoint_triplet), position_setpoint_triplet_fd, &position_setpoint_triplet);
		orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_fd, &vehicle_global_position);



		if (0.5f < distance_sensor_param.current_distance){
			/*
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
		if ( distance_sensor_param.current_distance < 10 && auxKike == 1){
			auxKike = 0;

			/*
			PX4_INFO("vehicle_control_mode:\n\tflag_armed\t%d\n\tflag_control_auto_enabled\t%d\n\tflag_control_manual_enabled\t%d\n\n",
			vehicle_control_mode_param.flag_armed,
			vehicle_control_mode_param.flag_control_auto_enabled,
			vehicle_control_mode_param.flag_control_manual_enabled
		);
		*/
		PX4_INFO("Follow Target previous is %f - %f ",position_setpoint_triplet.previous.lon, position_setpoint_triplet.previous.lat);
		PX4_INFO("Follow Target current is %f - %f ",position_setpoint_triplet.current.lon, position_setpoint_triplet.current.lat);

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

		int32_t newPoint[2];
		obtain_new_point(position_setpoint_triplet, newPoint, vehicle_global_position);

		PX4_INFO("New point is %d - %d ",newPoint[0], newPoint[1]);

		connect_mavlink();

		mavlink_mission_count_t mission_count;
		mavlink_mission_item_int_t * mission_items;

		receive_mission(&mission_count, &mission_items);

		//mission_items = (mavlink_mission_item_int_t*) malloc(4*sizeof(mavlink_mission_item_int_t));
		sleep(3);

		send_new_mission(mission_count, mission_items, newPoint, 0);

		sleep(3);

		cmd = {
			.timestamp = 0,
			.param5 = NAN,
			.param6 = NAN,
			.param1 = 1,
			.param2 = 4,
			.param3 = 4,
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


		h = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
		(void)orb_unadvertise(h);


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
		PX4_INFO("El mensaje creado es: \n\t	param1 %f,\n\t	param2 %f,\n\t	param3 %f,\n\t	param4 %f,\n\t	x %f,\n\t	y %f,\n\t	z %f,\n\t	seq %d,\n\t	command %d,\n\t	target_system %d,\n\t	target_component %d,\n\t	frame %d,\n\t	current %d,\n\t	autocontinue %d,\n\t	mission_type %d,\n", param1,
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

		//	h = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
		//	(void)orb_unadvertise(h);


		PX4_INFO("vehicle_control_mode:\n\tflag_armed\t%d\n\tflag_control_auto_enabled\t%d\n\tflag_control_manual_enabled\t%d\n\n",
		vehicle_control_mode_param.flag_armed,
		vehicle_control_mode_param.flag_control_auto_enabled,
		auxKike
	);

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

*/
			}
		}
	}
	PX4_INFO("GoodBye");
}

void SensorDistancia::obtain_new_point(struct position_setpoint_triplet_s position_setpoint_triplet, int32_t * newPoint, struct vehicle_global_position_s vehicle_global_position){

	int32_t x_previa = position_setpoint_triplet.previous.lon * 10000000;
	int32_t y_previa = position_setpoint_triplet.previous.lat * 10000000;

	int32_t x_next = position_setpoint_triplet.current.lon * 10000000;
	int32_t y_next = position_setpoint_triplet.current.lat * 10000000;

	int32_t x_current = vehicle_global_position.lon * 10000000;
	int32_t y_current = vehicle_global_position.lat * 10000000;

	PX4_INFO("lat y lon del vehiculo actualmente: %d - %d", x_current, y_current);

	// Se calcula el vector que une los dos puntos (previous y current)
	int32_t vector[2];
	vector[0] = x_next - x_previa;
	vector[1] = y_next - y_previa;

	// Se obtiene el vector perpendicular
	double vector_perpendicular[2];
	vector_perpendicular[0] = 1;
	vector_perpendicular[1] = (-1) * vector[0] / vector[1];

	//Escalamos el número para que el modulo sea el correcto x/d² and y/d²

	// Calculamos d para que la distancia sea 500
	double d = sqrt(( pow(vector_perpendicular[0],2) + pow(vector_perpendicular[1],2) )/25000);

	vector_perpendicular[0] = vector_perpendicular[0] / d;
	vector_perpendicular[1] = vector_perpendicular[0] / d;

	// Se obtiene el punto que pasa por previous y el vector paralelo
	newPoint[0] = rint(vector_perpendicular[0] + x_current);
	newPoint[1] = rint(vector_perpendicular[1] + y_current);
}


int SensorDistancia::send_mission_count(uint16_t count, uint8_t target_system, uint8_t target_component, uint8_t mission_type){

	mavlink_message_t msg;
	uint16_t len;
	uint8_t buf[BUFFER_LENGTH];

	const mavlink_mission_count_t wpc {
		.count = count,
		.target_system = target_system,
		.target_component = target_component,
		.mission_type = mission_type
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

mavlink_mission_count_t SensorDistancia::receive_mission_count(){
	ssize_t recsize;
	socklen_t fromlen;
	uint8_t buf[BUFFER_LENGTH];
	memset(buf, 0, BUFFER_LENGTH);
	int salir = 1;
	mavlink_mission_count_t mission_count;

	while (salir == 1){
		recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
		if (recsize > 0){
			// Something received - print out all bytes and parse packet

			mavlink_message_t msg;
			msg.sysid = 0;
			msg.compid = 0;
			msg.len = 0;
			msg.msgid = 0;

			mavlink_status_t status;

			for (int i = 0; i < recsize; ++i)
			{
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
				{
					if (msg.msgid == MAVLINK_MSG_ID_MISSION_COUNT) {

						mavlink_msg_mission_count_decode(&msg, &mission_count);
						printf("\nmission request:, \n \tCount: %d, \n \tMISSION_TYPE: %d, \n", mission_count.count,mission_count.mission_type);
						return mission_count;

					}
				}
			}
		}
	}
	return mission_count;
}


int SensorDistancia::send_request_list(uint8_t mission_type){

	mavlink_message_t msg;
	uint16_t len;
	uint8_t buf[BUFFER_LENGTH];

	const mavlink_mission_request_list_t request_list {
		.target_system = 1,
		.target_component = 1,
		.mission_type = mission_type
	};

	// Generamos mensaje
	mavlink_msg_mission_request_list_encode(255, 0, &msg, &request_list);
	len = mavlink_msg_to_send_buffer(buf, &msg);

	uint16_t bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr_in));
	if (bytes_sent != len){
		PX4_WARN("Error al enviar request_list");
		return -1;
	}
	return bytes_sent;
}


int SensorDistancia::send_item(float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z, uint16_t seq, uint16_t command,
	uint8_t target_system, uint8_t target_component, uint8_t frame, uint8_t current, uint8_t autocontinue, uint8_t mission_type ){

	mavlink_message_t msg;
	uint16_t len;
	uint8_t buf[BUFFER_LENGTH];

	PX4_INFO("El mensaje recibido en px4 sobre la mision es: \n\t	param1 %f,\n\t	param2 %f,\n\t	param3 %f,\n\t	param4 %f,\n\t	X %d,\n\t	y %d,\n\t	z %f,\n\t	seq %d,\n\t	command %d,\n\t	target_system %d,\n\t	target_component %d,\n\t	frame %d,\n\t	current %d,\n\t	autocontinue %d,\n\t	mission_type %d,\n",(double)	 param1,
	(double)param2,
	(double)param3,
	(double)param4,
	x,
	y,
	(double)z,
	seq,
	command,
	target_system,
	target_component,
	frame,
	current,
	autocontinue,
	mission_type);


	const mavlink_mission_item_int_t wp {
		.param1 = param1,
		.param2 = param2,
		.param3 = param3,
		.param4 = param4,
		.x = x,
		.y = y,
		.z = z,
		.seq = seq,
		.command = command,
		.target_system = 1,
		.target_component = 190,
		.frame = frame,
		.current = current,
		.autocontinue = autocontinue,
		.mission_type = 0
	};

	// Generamos mensaje
	mavlink_msg_mission_item_int_encode(255, 0, &msg, &wp);
	len = mavlink_msg_to_send_buffer(buf, &msg);

	uint16_t bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr_in));
	if (bytes_sent != len){
		PX4_WARN("Error al enviar count_mission");
		return -1;
	}
	return bytes_sent;
}

mavlink_mission_item_int_t SensorDistancia::receive_mission_item_int(){
	ssize_t recsize;
	socklen_t fromlen;
	uint8_t buf[BUFFER_LENGTH];
	memset(buf, 0, BUFFER_LENGTH);
	int salir = 1;
	mavlink_mission_item_int_t mission_item;

	while (salir == 1){
		recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
		if (recsize > 0){
			// Something received - print out all bytes and parse packet

			mavlink_message_t msg;
			msg.sysid = 0;
			msg.compid = 0;
			msg.len = 0;
			msg.msgid = 0;

			mavlink_status_t status;

			for (int i = 0; i < recsize; ++i)
			{
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
				{

					printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);

					if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT) {

						mavlink_msg_mission_item_int_decode(&msg, &mission_item);

						PX4_INFO("El mensaje recibido en px4 sobre la mision es: \n\t	param1 %f,\n\t	param2 %f,\n\t	param3 %f,\n\t	param4 %f,\n\t	X %d,\n\t	y %d,\n\t	z %f,\n\t	seq %d,\n\t	command %d,\n\t	target_system %d,\n\t	target_component %d,\n\t	frame %d,\n\t	current %d,\n\t	autocontinue %d,\n\t	mission_type %d,\n",(double)mission_item.param1,
						(double)mission_item.param2,
						(double)mission_item.param3,
						(double)mission_item.param4,
						mission_item.x,
						mission_item.y,
						(double)mission_item.z,
						mission_item.seq,
						mission_item.command,
						mission_item.target_system,
						mission_item.target_component,
						mission_item.frame,
						mission_item.current,
						mission_item.autocontinue,
						mission_item.mission_type);

						return mission_item;

					}
				}
			}
		}
	}
	return mission_item;
}

int SensorDistancia::send_new_mission(mavlink_mission_count_t mission_count ,mavlink_mission_item_int_t * items , int32_t * newWaypoint, uint16_t pos){

	// Enviamos count a para establecer nueva misión
	int leng_send_count = send_mission_count(4, 1, 1, 0);
	PX4_INFO("LENG SEND COUNT %d", leng_send_count);

	int err_recibir_request = recibir_request_int();
	PX4_INFO("REQUEST %d", err_recibir_request);
	int seq = 0;
	int count = 0;
	while (count < mission_count.count ){

		if (count == pos) {

			leng_send_count = send_item(items[count].param1, items[count].param2, items[count].param3, items[count].param4, newWaypoint[1] , newWaypoint[0] , items[count].z, seq, items[count].command, items[count].target_system, items[count].target_component , items[count].frame, 1,items[count].autocontinue, items[count].mission_type );
			PX4_INFO("LENG NEW SEND ITEM %d", leng_send_count);

			err_recibir_request = recibir_request_int();
			PX4_INFO("REQUEST ITEM ACK %d", err_recibir_request);
			seq ++;

		}

		leng_send_count = send_item(items[count].param1, items[count].param2, items[count].param3, items[count].param4, items[count].x , items[count].y , items[count].z, seq, items[count].command, items[count].target_system, items[count].target_component, items[count].frame, 0,items[count].autocontinue, items[count].mission_type );
		PX4_INFO("LENG SEND ITEM %d", leng_send_count);

		err_recibir_request = recibir_request_int();
		PX4_INFO("REQUEST ITEM %d", err_recibir_request);
		seq ++;


		count++;
	}

	return 0;
}


int SensorDistancia::receive_mission(mavlink_mission_count_t * mission_count, mavlink_mission_item_int_t ** items){


	// Enviamos count a para establecer nueva misión
	int leng_send_count = send_request_list(0);
	PX4_INFO("LENG REQUEST LIST %d", leng_send_count);

	*mission_count = receive_mission_count();
	PX4_INFO("RECEIVE REQUEST %d", mission_count->count);

	items[0] = (mavlink_mission_item_int_t*) malloc((mission_count->count)*sizeof(mavlink_mission_item_int_t));

	int count = 0;
	while( count < mission_count->count ) {

			leng_send_count = send_request_int(count, 1, 190, mission_count->mission_type);
			PX4_INFO("SEND REQUEST %d", leng_send_count);

			items[0][count] = receive_mission_item_int();
			PX4_INFO("Receive Item %d", count);

			count ++;
	}

	int send_ack = send_mission_ack(1,190, 0 ,mission_count->mission_type );
	PX4_INFO("SEND ACK %d", send_ack);

	//send_request(count, 1, 190, mission_count->mission_type);

	return 0;
}


int SensorDistancia::connect_mavlink(){


	sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

	// Conectamos con localhost
	char target_ip[10];
	strcpy(target_ip, "127.0.0.1");

	// Para recibir
	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(14551);

	// Para enviar
	memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcAddr.sin_port = htons(14557);

	if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr))){
		PX4_INFO("Error Bind");
		return -1;
	}
	return 0;
}

/*
int SensorDistancia::recibir_request(){
	ssize_t recsize;
	socklen_t fromlen;
	uint8_t buf[BUFFER_LENGTH];
	memset(buf, 0, BUFFER_LENGTH);
//	unsigned int temp = 0;
	int salir = 1;

	while (salir == 1){
		recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
		if (recsize > 0){
			// Something received - print out all bytes and parse packet

			mavlink_message_t msg;
			msg.sysid = 0;
			msg.compid = 0;
			msg.len = 0;
			msg.msgid = 0;

			mavlink_status_t status;

			for (int i = 0; i < recsize; ++i)
			{
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
				{

					if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST) {

						mavlink_mission_request_t mission_request;
						mavlink_msg_mission_request_decode(&msg, &mission_request);
						printf("\nmission request:, \n \tSEQ: %d, \n \tMISSION_TYPE: %d, \n", mission_request.seq,mission_request.mission_type);
						return 0;

					}
				}
			}
		}
	}
	return 0;
}
*/
int SensorDistancia::recibir_request_int(){
	ssize_t recsize;
	socklen_t fromlen;
	uint8_t buf[BUFFER_LENGTH];
	memset(buf, 0, BUFFER_LENGTH);
//	unsigned int temp = 0;
	int salir = 1;

	while (salir == 1){
		recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
		if (recsize > 0){
			// Something received - print out all bytes and parse packet

			mavlink_message_t msg;
			msg.sysid = 0;
			msg.compid = 0;
			msg.len = 0;
			msg.msgid = 0;

			mavlink_status_t status;

			for (int i = 0; i < recsize; ++i)
			{
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
				{

					if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT) {

						mavlink_mission_request_int_t mission_request;
						mavlink_msg_mission_request_int_decode(&msg, &mission_request);
						printf("\nmission request:, \n \tSEQ: %d, \n \tMISSION_TYPE: %d, \n", mission_request.seq,mission_request.mission_type);
						return 0;

					}
				}
			}
		}
	}
	return 0;
}

int SensorDistancia::send_request_int(uint16_t seq, uint8_t target_system, uint8_t target_component, uint8_t mission_type){

	mavlink_message_t msg;
	uint16_t len;
	uint8_t buf[BUFFER_LENGTH];

	const mavlink_mission_request_int_t request_int {
		.seq = seq,
		.target_system = target_system,
		.target_component = target_component,
		.mission_type = mission_type
	};

	// Generamos mensaje
	mavlink_msg_mission_request_int_encode(255, 0, &msg, &request_int);
	len = mavlink_msg_to_send_buffer(buf, &msg);

	uint16_t bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr_in));
	if (bytes_sent != len){
		PX4_WARN("Error al enviar request_int");
		return -1;
	}
	return bytes_sent;
}

int SensorDistancia::send_request(uint16_t seq, uint8_t target_system, uint8_t target_component, uint8_t mission_type){

	mavlink_message_t msg;
	uint16_t len;
	uint8_t buf[BUFFER_LENGTH];

	const mavlink_mission_request_t request {
		.seq = seq,
		.target_system = target_system,
		.target_component = target_component,
		.mission_type = mission_type
	};

	// Generamos mensaje
	mavlink_msg_mission_request_encode(255, 0, &msg, &request);
	len = mavlink_msg_to_send_buffer(buf, &msg);

	uint16_t bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr_in));
	if (bytes_sent != len){
		PX4_WARN("Error al enviar request");
		return -1;
	}
	return bytes_sent;
}


int SensorDistancia::recibir_ack(){
	ssize_t recsize;
	socklen_t fromlen;
	uint8_t buf[BUFFER_LENGTH];
	memset(buf, 0, BUFFER_LENGTH);
	//unsigned int temp = 0;
	int salir = 1;

	while (salir == 1){
		recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
		if (recsize > 0){
			// Something received - print out all bytes and parse packet

			mavlink_message_t msg;
			msg.sysid = 0;
			msg.compid = 0;
			msg.len = 0;
			msg.msgid = 0;

			mavlink_status_t status;
			//printf("Bytes Received: %d\nDatagram: ", (int)recsize);

			for (int i = 0; i < recsize; ++i)
			{
				//temp = buf[i];
				//printf("%02x ", (unsigned char)temp);
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
				{
					// Packet received
					//printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);

					if (msg.msgid == MAVLINK_MSG_ID_MISSION_ACK) {


						mavlink_mission_request_t mission_request;
						mavlink_msg_mission_request_decode(&msg, &mission_request);
						printf("\nmission request:, \n \tSEQ: %d, \n \tMISSION_TYPE: %d, \n", mission_request.seq,mission_request.mission_type);
						return 0;

					}
				}
			}
			//printf("\n");
		}
	}
	return 0;
}

int SensorDistancia::send_mission_ack(uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mission_type){

	mavlink_message_t msg;
	uint16_t len;
	uint8_t buf[BUFFER_LENGTH];

	const mavlink_mission_ack_t mission_ack {
		.target_system = target_system,
		.target_component = target_component,
		.type = type,
		.mission_type = mission_type
	};

	// Generamos mensaje
	mavlink_msg_mission_ack_encode(255, 0, &msg, &mission_ack);
	len = mavlink_msg_to_send_buffer(buf, &msg);

	uint16_t bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr_in));
	if (bytes_sent != len){
		PX4_WARN("Error al enviar count_mission");
		return -1;
	}
	return bytes_sent;
}



/*
EJEMPLO DE RECIBIR A TRAVÉS DE MAVLINK

ssize_t recsize;
socklen_t fromlen;
int i = 0;
//int success = 0;
unsigned int temp = 0;
uint8_t buf[BUFFER_LENGTH];

for (;;)
{

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



*/

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
