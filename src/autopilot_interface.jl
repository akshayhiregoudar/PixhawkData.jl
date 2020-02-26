using Cxx
using Libdl

const path_to_lib = "/home/jarv1s/pixhawk_sensor_data"
#const path_to_header = joinpath(@__DIR__, "..", "deps", "usr", "include")

addHeaderDir(path_to_lib, kind=C_System)
Libdl.dlopen(path_to_lib * "/libPixhawkData.so", Libdl.RTLD_GLOBAL)

cxxinclude("serial_port.h")
cxxinclude("autopilot_interface.h")

cxx"#include <signal.h>"
cxx"#include <time.h>"
cxx"#include <sys/time.h>"

cxx"#include <mavlink/include/mavlink/v2.0/common/mavlink.h>"

cxx"""
uint64_t get_time_usec();
void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp);
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp);
void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp);
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp);
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp);

void* start_autopilot_interface_read_thread(void *args);
void* start_autopilot_interface_write_thread(void *args);
"""

get_time_usec() = @cxx get_time_usec()
set_position()= @cxx set_position()
set_velocity()= @cxx set_velocity()
set_acceleration()= @cxx set_acceleration()
set_yaw()= @cxx set_yaw()
set_yaw_rate()= @cxx set_yaw_rate()

start_autopilot_interface_read_thread() = @cxx start_autopilot_interface_read_thread()
start_autopilot_interface_write_thread() = @cxx start_autopilot_interface_write_thread()

#=cxx"""
       struct Time_Stamps
       {
               Time_Stamps()
               {
                       reset_timestamps();
               }

               uint64_t heartbeat;
               uint64_t sys_status;
               uint64_t battery_status;
               uint64_t radio_status;
               uint64_t local_position_ned;
               uint64_t global_position_int;
               uint64_t position_target_local_ned;
               uint64_t position_target_global_int;
               uint64_t highres_imu;
               uint64_t attitude;

               void
               reset_timestamps()
               {
                       heartbeat = 0;
                       sys_status = 0;
                       battery_status = 0;
                       radio_status = 0;
                       local_position_ned = 0;
                       global_position_int = 0;
                       position_target_local_ned = 0;
                       position_target_global_int = 0;
                       highres_imu = 0;
                       attitude = 0;
               }

       };
"""=#

Time_Stamps() = @cxx Time_Stamps()

#=cxx"""
struct Mavlink_Messages {

	int sysid;
	int compid;


	mavlink_heartbeat_t heartbeat;

	mavlink_sys_status_t sys_status;

	mavlink_battery_status_t battery_status;

	mavlink_radio_status_t radio_status;

	mavlink_local_position_ned_t local_position_ned;

	mavlink_global_position_int_t global_position_int;

	mavlink_position_target_local_ned_t position_target_local_ned;

	mavlink_position_target_global_int_t position_target_global_int;

	mavlink_highres_imu_t highres_imu;

	mavlink_attitude_t attitude;

	Time_Stamps time_stamps;

	void
	reset_timestamps()
	{
		time_stamps.reset_timestamps();
	}

};
"""=#

Mavlink_Messages() = @cxx Mavlink_Messages()

#=cxx"""
class Autopilot_Interface
{

public:

	Autopilot_Interface();
	Autopilot_Interface(Serial_Port *serial_port_);
	~Autopilot_Interface();

	char reading_status;
	char writing_status;
	char control_status;
    uint64_t write_count;

    int system_id;
	int autopilot_id;
	int companion_id;

	Mavlink_Messages current_messages;
	mavlink_set_position_target_local_ned_t initial_position;

	void update_setpoint(mavlink_set_position_target_local_ned_t setpoint);
	void read_messages();
	int  write_message(mavlink_message_t message);

	void enable_offboard_control();
	void disable_offboard_control();

	void start();
	void stop();

	void start_read_thread();
	void start_write_thread(void);

	void handle_quit( int sig );


private:

	Serial_Port *serial_port;

	bool time_to_exit;

	pthread_t read_tid;
	pthread_t write_tid;

	mavlink_set_position_target_local_ned_t current_setpoint;

	void read_thread();
	void write_thread(void);

	int toggle_offboard_control( bool flag );
	void write_setpoint();

};
"""=#

# Autopilot_Interface = @cxxnew Autopilot_Interface()


cxx"""
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}
"""

get_time_usec() = @cxx get_time_usec()

cxx"""
void
set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.x   = x;
	sp.y   = y;
	sp.z   = z;
}
"""

set_position() = @cxx set_position()

cxx"""
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;
}
"""

set_velocity() = @cxx set_velocity()

cxx"""
void
set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

	// NOT IMPLEMENTED
	fprintf(stderr,"set_acceleration doesn't work yet \n");
	throw 1;


	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;
}
"""

set_acceleration() = @cxx set_acceleration()

cxx"""
void
set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

	sp.yaw  = yaw;
}
"""

set_yaw() = @cxx set_yaw()

cxx"""
void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}
"""

set_yaw_rate() = @cxx set_yaw_rate()

cxx"""
Autopilot_Interface::
Autopilot_Interface(Serial_Port *serial_port_)
{
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	serial_port = serial_port_; // serial port management object

}
"""

Autopilot_Interface() = @cxx Autopilot_Interface()

cxx"""
Autopilot_Interface::
~Autopilot_Interface()
{}
"""

Autopilot_Interface1() = @cxx ~Autopilot_Interface()

cxx"""
void
Autopilot_Interface::
update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	current_setpoint = setpoint;
}
"""

update_setpoint() = @cxx update_setpoint()

cxx"""
void
Autopilot_Interface::
read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		mavlink_message_t message;
		success = serial_port->read_message(message);


		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}

				default:
				{
					// printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
//				this_timestamps.battery_status             &&
//				this_timestamps.radio_status               &&
//				this_timestamps.local_position_ned         &&
//				this_timestamps.global_position_int        &&
//				this_timestamps.position_target_local_ned  &&
//				this_timestamps.position_target_global_int &&
//				this_timestamps.highres_imu                &&
//				this_timestamps.attitude                   &&
				this_timestamps.sys_status
				;

		// give the write thread time to use the port
		if ( writing_status > false ) {
			usleep(100); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}
"""

read_messages() = @cxx read_messages()

cxx"""
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = serial_port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}
"""

write_message() = @cxx write_message()

cxx"""
void
Autopilot_Interface::
write_setpoint()
{

	// pull from position target
	mavlink_set_position_target_local_ned_t sp = current_setpoint;

	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;


	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);



	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	return;
}
"""

write_setpoint() = @cxx write_setpoint()

cxx"""
void
Autopilot_Interface::
enable_offboard_control()
{
	// Should only send this command once
	if ( control_status == false )
	{
		//printf("ENABLE OFFBOARD MODE\n");


		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		//printf("\n");

	} // end: if not offboard_status

}
"""

enable_offboard_control() = @cxx enable_offboard_control()

cxx"""
void
Autopilot_Interface::
disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == true )
	{
		//printf("DISABLE OFFBOARD MODE\n");


		// Sends the command to stop off-board
		int success = toggle_offboard_control( false );

		// Check the command was written
		if ( success )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		//printf("\n");

	} // end: if offboard_status

}
"""

disable_offboard_control() = @cxx disable_offboard_control()

cxx"""
int
Autopilot_Interface::
toggle_offboard_control( bool flag )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = serial_port->write_message(message);

	// Done!
	return len;
}
"""

toggle_offboard_control() = @cxx toggle_offboard_control()

cxx"""
void
Autopilot_Interface::
start()
{
	int result;


	if ( serial_port->status != 1 ) // SERIAL_PORT_OPEN
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		throw 1;
	}



	//printf("START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	//printf("\n");



	//printf("CHECK FOR MESSAGES\n");

	while ( not current_messages.sysid )
	{
		if ( time_to_exit )
			return;
		usleep(500000); // check at 2Hz
	}

	//printf("Found\n");

	// now we know autopilot is sending messages
	//printf("\n");


	// System ID
	//if ( not system_id )
	{
		//system_id = current_messages.sysid;
		//printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	//if ( not autopilot_id )
	{
		//autopilot_id = current_messages.compid;
		//printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		//printf("\n");
	}


	// Wait for initial position ned
	while ( not ( current_messages.time_stamps.local_position_ned &&
				  current_messages.time_stamps.attitude            )  )
	{
		if ( time_to_exit )
			return;
		usleep(500000);
	}

	// copy initial position ned
	Mavlink_Messages local_data = current_messages;
	initial_position.x        = local_data.local_position_ned.x;
	initial_position.y        = local_data.local_position_ned.y;
	initial_position.z        = local_data.local_position_ned.z;
	initial_position.vx       = local_data.local_position_ned.vx;
	initial_position.vy       = local_data.local_position_ned.vy;
	initial_position.vz       = local_data.local_position_ned.vz;
	initial_position.yaw      = local_data.attitude.yaw;
	initial_position.yaw_rate = local_data.attitude.yawspeed;

	//printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
	//printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
	//printf("\n");

	// we need this before starting the write thread


	//printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if ( result ) throw result;

	// wait for it to be started
	while ( not writing_status )
		usleep(100000); // 10Hz

	// now we're streaming setpoint commands
	//printf("\n");


	// Done!
	return;

}
"""

start() = @cxx start()

cxx"""
void
Autopilot_Interface::
stop()
{
	//printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	//printf("\n");

	// still need to close the serial_port separately
}
"""

stop() = @cxx stop()

cxx"""
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}
"""

start_read_thread() = @cxx start_read_thread()

cxx"""
void
Autopilot_Interface::
start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}
"""

start_write_thread() = @cxx start_write_thread()

cxx"""
void
Autopilot_Interface::
handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}
"""

handle_quit() = @cxx handle_quit()

cxx"""
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}
"""

read_thread() = @cxx read_thread()

cxx"""
void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
	writing_status = 2;

	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx       = 0.0;
	sp.vy       = 0.0;
	sp.vz       = 0.0;
	sp.yaw_rate = 0.0;

	// set position target
	current_setpoint = sp;

	// write a message and signal writing
	write_setpoint();
	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( !time_to_exit )
	{
		usleep(250000);   // Stream at 4Hz
		write_setpoint();
	}

	// signal end
	writing_status = false;

	return;

}
"""

write_thread() = @cxx write_thread()

cxx"""
void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}
"""

start_autopilot_interface_read_thread() = @cxx start_autopilot_interface_read_thread()

cxx"""
void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}
"""

start_autopilot_interface_write_thread() = @cxx start_autopilot_interface_write_thread()
