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


#include <lib/ecl/geo/geo.h>



#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <mavlink/mavlink_mission.h>
#include <mavlink/mavlink_main.h>
#include <mavlink/mavlink_command_sender.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

//#include <mc_pos_control/mc_pos_control_main.cpp>
#include <uORB/topics/sensor_baro.h>	// Altura por barometro
#include <uORB/topics/position_setpoint_triplet.h>	// Altura por barometro
#include <uORB/topics/vehicle_attitude.h>	// velocidad (rollspeed, pitchspeed, yawspeed)
#include <uORB/topics/distance_sensor.h>	// sensor de distancia (min_distance, max_distance, current_distance, coveriance ,MAV_DISTANCE_SENSOR_LASER)
#include <uORB/topics/vehicle_control_mode.h>	// sensor de distancia (min_distance, max_distance, current_distance, coveriance ,MAV_DISTANCE_SENSOR_LASER)
#include <uORB/topics/vehicle_global_position.h>	// altura (z)
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_attitude.h>
#include <drivers/drv_hrt.h>


#define NO_DANGER 0
#define DANGER 1
#define AVOIDANCE 2
#define DISTANCIA_PELIGRO 3
#define DISTANCIA_NO_PELIGRO 5
#define FRONT 0
#define LEFT 1
#define BACK 2
#define RIGHT 3


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

	void obtain_new_point(struct position_setpoint_triplet_s position_setpoint_triplet, double * lat, double * lon, struct vehicle_global_position_s vehicle_global_position, int orientation);
	void task_main();
	void execute_obstacle_avoidance(struct vehicle_command_s cmd, orb_advert_t cmd_pub, int orientation);
	bool lectura_suelo(int vehicle_attitude_fd, double altura_real, double distancia_real);
	void continue_mission(struct vehicle_command_s cmd, orb_advert_t cmd_pub);
	void pause_mission(struct vehicle_command_s cmd, orb_advert_t cmd_pub);

private:
	bool		_task_should_exit = false;		/**< if true, task should exit */
	int		_control_task = -1;			/**< task handle for task */
	int vehicle_status_fd;
	int position_setpoint_triplet_fd;
	int vehicle_global_position_fd;
	int fase_de_lectura;
	int distancia_no_peligro;
	int distancia_peligro;
	uint64_t start_time;
	int no_avoidance;
	int warning_left;
	int warning_right;
	int warning_back;
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

	int sensor_sub_fd_sensor = orb_subscribe(ORB_ID(distance_sensor));
	int vehicle_control_mode_fd = orb_subscribe(ORB_ID(vehicle_control_mode));
	vehicle_status_fd = orb_subscribe(ORB_ID(vehicle_status));
	position_setpoint_triplet_fd = orb_subscribe(ORB_ID(position_setpoint_triplet));
	vehicle_global_position_fd = orb_subscribe(ORB_ID(vehicle_global_position));
	int  vehicle_local_position_setpoint_fd = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	int  vehicle_attitude_fd = orb_subscribe(ORB_ID(vehicle_attitude));


	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd_sensor, 100);
	orb_set_interval(vehicle_control_mode_fd, 100);
	orb_set_interval(vehicle_status_fd, 100);
	orb_set_interval(position_setpoint_triplet_fd, 100);
	orb_set_interval(vehicle_global_position_fd, 100);
	orb_set_interval(vehicle_local_position_setpoint_fd, 100);
	orb_set_interval(vehicle_attitude_fd, 100);


	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[7]; /* = {
		{ .fd = sensor_sub_fd_sensor, .events = POLLIN },
		{ .fd = vehicle_control_mode_fd, .events = POLLIN },
		{ .fd = vehicle_status_fd, .events = POLLIN },
		{ .fd = position_setpoint_triplet_fd, .events = POLLIN },
		{ .fd = vehicle_global_position_fd, .events = POLLIN },
		{ .fd = vehicle_local_position_setpoint_fd, .events = POLLIN },
		{ .fd = vehicle_attitude_fd, .events = POLLIN }
	}; */
	fds[0].fd = sensor_sub_fd_sensor;
	fds[0].events = POLLIN;

	fds[1].fd = vehicle_control_mode_fd;
	fds[1].events = POLLIN;

	fds[2].fd = vehicle_status_fd;
	fds[2].events = POLLIN;

	fds[3].fd = position_setpoint_triplet_fd;
	fds[3].events = POLLIN;

	fds[4].fd = vehicle_global_position_fd;
	fds[4].events = POLLIN;

	fds[5].fd = vehicle_local_position_setpoint_fd;
	fds[5].events = POLLIN;

	fds[6].fd = vehicle_attitude_fd;
	fds[6].events = POLLIN;

	// publisher for commands
	struct vehicle_command_s cmd;
	memset(&cmd, 0, sizeof(cmd));
	orb_advert_t cmd_pub = orb_advertise(ORB_ID(vehicle_command), &cmd);

	int error_counter = 0;
	//int counter_prueba = 0;
	fase_de_lectura = NO_DANGER;
	int contador_lecturas_no_peligrosas = 0;
	no_avoidance = true;
	warning_left = false;
	warning_right = false;
	warning_back = false;

	while(!_task_should_exit){
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 250);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {

				/* obtained data for the first file descriptor */
				struct distance_sensor_s distance_sensor_param;
				struct vehicle_control_mode_s vehicle_control_mode_param;
				struct vehicle_local_position_setpoint_s vehicle_local_position_setpoint_param;

				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(distance_sensor), sensor_sub_fd_sensor, &distance_sensor_param);
				orb_copy(ORB_ID(vehicle_control_mode), vehicle_control_mode_fd, &vehicle_control_mode_param);
				orb_copy(ORB_ID(vehicle_local_position_setpoint), vehicle_local_position_setpoint_fd, &vehicle_local_position_setpoint_param);

				/*
				struct vehicle_status_s status;
				orb_copy(ORB_ID(vehicle_status), vehicle_status_fd, &status);
				PX4_INFO("ESTADO DEL DRONE %d",status.nav_state);
				*/


				double speed = sqrt(pow(vehicle_local_position_setpoint_param.vx,2)+pow(vehicle_local_position_setpoint_param.vy,2));
				// PX4_INFO("distance : %f",(double)distance_sensor_param.current_distance);
				// PX4_INFO("speed: %f",speed);
				distancia_no_peligro = DISTANCIA_NO_PELIGRO + rint(speed);
				distancia_peligro = DISTANCIA_PELIGRO + rint(speed);


				switch (fase_de_lectura) {

					case NO_DANGER:
					if (distance_sensor_param.current_distance < distancia_no_peligro && distance_sensor_param.orientation == FRONT){
						fase_de_lectura = DANGER;
					}
					break;

					case DANGER:
					if (distance_sensor_param.current_distance > distancia_no_peligro && distance_sensor_param.orientation == FRONT){
						// Si no detectamos lecturas positivas
						contador_lecturas_no_peligrosas++;
						if(contador_lecturas_no_peligrosas > 3){
							contador_lecturas_no_peligrosas = 0;
							fase_de_lectura = NO_DANGER;
						}
					} else {
						// Comprobamos si la lectura es de suelo
						bool suelo = lectura_suelo(vehicle_attitude_fd, vehicle_local_position_setpoint_param.z, distance_sensor_param.current_distance);
						if (distance_sensor_param.current_distance < distancia_peligro && !suelo && distance_sensor_param.orientation == FRONT) {
							pause_mission(cmd, cmd_pub);
							fase_de_lectura = AVOIDANCE;
						}
					}
					break;

					case AVOIDANCE:

					// Miramos si hay un objeto en la direccion donde vamos a realizar la evasión
					if (no_avoidance){
						switch (distance_sensor_param.orientation) {
							case LEFT:
							if (distance_sensor_param.current_distance > distancia_peligro) {
								execute_obstacle_avoidance(cmd, cmd_pub, LEFT);
								start_time = hrt_absolute_time();
								no_avoidance = false;
								warning_left = false;
								warning_right = false;
							} else {
								warning_left = true;
							}
							break;

							case RIGHT:
							if (distance_sensor_param.current_distance > distancia_peligro) {
								execute_obstacle_avoidance(cmd, cmd_pub, RIGHT);
								start_time = hrt_absolute_time();
								no_avoidance = false;
								warning_left = false;
								warning_right = false;

							} else {
								warning_right = true;
							}
							break;

							case BACK:
							if (warning_left && warning_right){
								if (distance_sensor_param.current_distance > distancia_peligro) {
									execute_obstacle_avoidance(cmd, cmd_pub, BACK);
									start_time = hrt_absolute_time();
									no_avoidance = false;
									warning_left = false;
									warning_right = false;

								} else {
									warning_back = true;
								}
							}
							break;

						}
					} else {
						uint64_t diff_time = hrt_absolute_time() - start_time;


						// Tiempo en recontinuar la mision
						if (diff_time > 5000000) {
							no_avoidance = true;
							continue_mission(cmd, cmd_pub);
							fase_de_lectura = NO_DANGER;
						}
					}

					if ( warning_left && warning_right && warning_back ){
						PX4_ERR("Imposible esquivar");
					}

					/*
					else { // Mientras la evasión está activa, comprobamos sensor
						bool suelo = lectura_suelo(vehicle_attitude_fd, vehicle_local_position_setpoint_param.z, distance_sensor_param.current_distance);
						if (distance_sensor_param.current_distance < distancia_peligro && !suelo) {
							pause_mission(cmd, cmd_pub);
							no_avoidance = true;
						}
					}
					*/
					break;
				}
			}
		}
	}
}

void SensorDistancia::execute_obstacle_avoidance(struct vehicle_command_s cmd, orb_advert_t cmd_pub, int orientation){

	struct vehicle_status_s status;
	struct position_setpoint_triplet_s position_setpoint_triplet;
	struct vehicle_global_position_s vehicle_global_position;

	orb_copy(ORB_ID(vehicle_status), vehicle_status_fd, &status);
	orb_copy(ORB_ID(position_setpoint_triplet), position_setpoint_triplet_fd, &position_setpoint_triplet);
	orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_fd, &vehicle_global_position);


	double lat;
	double lon;
	obtain_new_point(position_setpoint_triplet, &lat, &lon , vehicle_global_position, orientation);

	cmd = {
		.timestamp = hrt_absolute_time(),
		.param5 = lat,
		.param6 = lon,
		.param1 = -1.0,
		.param2 = 1,
		.param3 = 0,
		.param4 = NAN,
		.param7 =  vehicle_global_position.alt, // altura
		.command = 192,
		.target_system = 1,
		.target_component = 1,
		.source_system = 1,
		.source_component = 1,
		.confirmation = 0,
		.from_external = false
	};

	orb_publish(ORB_ID(vehicle_command), cmd_pub, &cmd);

	// Esperamos lo suficiente para que el dron realice el giro
	sleep(3);

}

void SensorDistancia::continue_mission(struct vehicle_command_s cmd, orb_advert_t cmd_pub){

	struct vehicle_status_s status;
	orb_copy(ORB_ID(vehicle_status), vehicle_status_fd, &status);

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


	orb_publish(ORB_ID(vehicle_command), cmd_pub, &cmd);
}

void SensorDistancia::pause_mission(struct vehicle_command_s cmd, orb_advert_t cmd_pub){

	struct vehicle_global_position_s vehicle_global_position;
	orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_fd, &vehicle_global_position);

	struct vehicle_status_s status;
	orb_copy(ORB_ID(vehicle_status), vehicle_status_fd, &status);

	cmd = {
		.timestamp = 0,
		.param5 = NAN,
		.param6 = NAN,
		.param1 = -1,
		.param2 = 1,
		.param3 = 0,
		.param4 = NAN,
		.param7 = NAN,
		.command = 192,
		.target_system = status.system_id,
		.target_component = status.component_id,
		.source_system = 1,
		.source_component = 2,
		.confirmation = 1,
		.from_external = true
	};


	orb_publish(ORB_ID(vehicle_command), cmd_pub, &cmd);
}

void SensorDistancia::obtain_new_point(struct position_setpoint_triplet_s position_setpoint_triplet, double * lat, double * lon, struct vehicle_global_position_s vehicle_global_position, int orientation){


	// PX4_INFO("Follow Target previous is %f - %f ",position_setpoint_triplet.current.lat, position_setpoint_triplet.current.lon);
	// PX4_INFO("Position global is %f - %f ",vehicle_global_position.lat, vehicle_global_position.lon);


	// calculo el vector normal a los dos puntos
	double vector_normal [2];
	vector_normal[0] = 0;
	vector_normal[1] = 0;

	switch (orientation) {
		case LEFT:
		vector_normal[0] = position_setpoint_triplet.current.lon - vehicle_global_position.lon;
		vector_normal[1] = (position_setpoint_triplet.current.lat - vehicle_global_position.lat) * (-1);
		break;
		case RIGHT:
		vector_normal[0] = (position_setpoint_triplet.current.lon - vehicle_global_position.lon) * (-1);
		vector_normal[1] = position_setpoint_triplet.current.lat - vehicle_global_position.lat;
		break;
		case BACK:
		vector_normal[0] = (position_setpoint_triplet.current.lat - vehicle_global_position.lat) * (-1);
		vector_normal[1] = (position_setpoint_triplet.current.lon - vehicle_global_position.lon) * (-1);
		break;
	}


	// Calculo el vector unitario
	double length = sqrt(pow(vector_normal[0],2) + pow(vector_normal[1],2));
	vector_normal[0] = vector_normal[0] / length;
	vector_normal[1] = vector_normal[1] / length;

	// Escalo el vector
	vector_normal[0] = vector_normal[0] * 0.0001;
	vector_normal[1] = vector_normal[1] * 0.0001;

	*lat = vector_normal[0] + vehicle_global_position.lat;
	*lon = vector_normal[1] + vehicle_global_position.lon;


}

bool SensorDistancia::lectura_suelo(int vehicle_attitude_fd, double altura_real, double distancia_real){

	// Obtengo el ángulo para comprobar si es un objeto o es el suelo
	struct vehicle_attitude_s vehicle_attitude_param;
	orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_fd, &vehicle_attitude_param);

	float roll;
	float pitch;
	float yaw;



	mavlink_quaternion_to_euler(vehicle_attitude_param.q,&roll, &pitch, &yaw);
	double altura_predecida = sin(pitch) * distancia_real;

	/*
	PX4_INFO("PITCH %f",(double)pitch);
	PX4_INFO("ALTURA %f",altura_real);
	PX4_INFO("ALTURA CALCULADA %f",altura_predecida);
	*/

	if ( (altura_predecida + 0.5) > altura_real ){
		return false;
	} else {
		return true;
	}
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
