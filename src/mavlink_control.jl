using Cxx
using Libdl

#const path_to_lib = "/home/jarv1s/pixhawk_sensor_data"
#const path_to_header = joinpath(@__DIR__, "..", "deps", "usr", "include")

#addHeaderDir(path_to_lib, kind=C_System)
#Libdl.dlopen(path_to_lib * "/libPixhawkData.so", Libdl.RTLD_GLOBAL)

cxx"#include <iostream>"
cxx"#include <iostream>"
cxx"#include <stdio.h>"
cxx"#include <cstdlib>"
cxx"#include <unistd.h>"
cxx"#include <cmath>"
cxx"#include <string.h>"
cxx"#include <inttypes.h>"
cxx"#include <fstream>"
cxx"#include <signal.h>"
cxx"#include <time.h>"
cxx"#include <sys/time.h>"

cxx"#include <mavlink/include/mavlink/v2.0/common/mavlink.h>"

cxxinclude("autopilot_interface.h")
cxxinclude("serial_port.h")
cxxinclude("mavlink_control.h")

cxx"""
    int main(int argc, char **argv);
    int top(int argc, char **argv);

    void commands(Autopilot_Interface &autopilot_interface);
    void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);

    void quit_handler( int sig );
"""

cxx"""
int
top (int argc, char **argv)
{
#ifdef __APPLE__
	char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
	char *uart_name = (char*)"/dev/ttyUSB0";
#endif
	int baudrate = 57600;

	parse_commandline(argc, argv, uart_name, baudrate);

	Serial_Port serial_port(uart_name, baudrate);

	Autopilot_Interface autopilot_interface(&serial_port);

	serial_port_quit         = &serial_port;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT,quit_handler);

	serial_port.start();
	autopilot_interface.start();

	commands(autopilot_interface);

	autopilot_interface.stop();
	serial_port.stop();

	return 0;
}
"""

top() = @cxx top()

cxx"""
void
commands(Autopilot_Interface &api)
{
	Mavlink_Messages messages = api.current_messages;

	mavlink_highres_imu_t imu = messages.highres_imu;

	printf("acc_x =  % f, acc_y = %f, acc_z = %f \n", imu.xacc, imu.yacc, imu.zacc); // (m/s^2)
	printf("gyro_x = % f, gyro_y = %f, gyro_z = %f \n", imu.xgyro, imu.ygyro, imu.zgyro); // (rad/s)
	printf("mag_x = % f, mag_y = %f, mag_z = %f \n", imu.xmag, imu.ymag, imu.zmag); // (Ga)

	return;

}
"""

commands() = @cxx commands()

cxx"""
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}

	return;
}
"""

parse_commandline() = @cxx parse_commandline()

cxx"""
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// serial port
	try {
		serial_port_quit->handle_quit(sig);
	}
	catch (int error){}

	// end program here
	exit(0);

}
"""

quit_handler() = @cxx quit_handler()

cxx"""
int
main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}
"""

main() = @cxx main()
