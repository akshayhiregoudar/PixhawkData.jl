__precompile__(false)

module PixhawkData

using Cxx
using Libdl

const depfile = joinpath(@__DIR__, "..", "deps", "deps.jl")
if isfile(depfile)
    include(depfile)
else
    error("libPixhawkData is not properly installed. Please run Pkg.build(\"PixhawkData\") to rectify this error.")
end

const path_to_header = joinpath(@__DIR__, "..", "deps", "usr", "include")

function __init__()
    addHeaderDir(path_to_header, kind=C_System)
    Libdl.dlopen(libpixhawkdata, Libdl.RTLD_GLOBAL)

    cxx"#include <iostream>"
    cxx"#include <vector>"
    cxx"#include <tuple>"
end

export
    top,
    commands,
    parse_commandline,
    quit_handler,
    main

include("mavlink_control.jl")

export
    Serial_Port,
    Serial_Port1,
    initialize_defaults,
    read_message,
    write_message,
    open_serial,
    close_serial,
    start,
    stop,
    handle_quit,
    _open_port,
    _setup_port,
    _read_port,
    _write_port

include("serial_port.jl")

export
    get_time_usec,
    set_position,
    set_velocity,
    set_acceleration,
    set_yaw,
    set_yaw_rate
    start_autopilot_interface_read_thread,
    start_autopilot_interface_write_thread,
    Time_Stamps,
    Mavlink_Messages,
    get_time_usec,
    set_position,
    set_velocity,
    set_acceleration,
    set_yaw,
    set_yaw_rate,
    Autopilot_Interface,
    Autopilot_Interface1,
    update_setpoint,
    read_messages,
    write_messages,
    write_setpoint,
    enable_offboard_control,
    disable_offboard_control,
    toggle_offboard_control,
    start,
    stop,
    start_read_thread,
    start_write_thread,
    handle_quit,
    read_thread,
    write_thread,
    start_autopilot_interface_read_thread,
    start_autopilot_interface_write_thread

include("autopilot_interface.jl")

end
