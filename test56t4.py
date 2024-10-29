#!/usr/bin/env python3
import subprocess
import time
import signal
import sys
import os
import tempfile
import textwrap

# Configuration
SOURCE_IP = "192.168.1.193"
SOURCE_PORT = "14551"
DEST_PORT1 = "14552"
DEST_PORT2 = "14553"
LOCALHOST = "127.0.0.1"
BAUDRATE = "57600"  # Adjust if necessary
AIRCRAFT_NAME = "MyAircraft"
LOGFILE = "mavproxy_forward.log"

# Custom MAVProxy Module Code for Command Prioritization
CMD_PRIORITY_MODULE_CODE = textwrap.dedent("""
    from pymavlink import mavutil
    from MAVProxy.modules.lib import mp_module
    import threading

    class CommandPriorityModule(mp_module.MPModule):
        def __init__(self, mpstate):
            super(CommandPriorityModule, self).__init__(mpstate, "cmd_priority")
            self.add_command('cmd_priority', self.cmd_priority, "Handle command prioritization")
            # Setup listeners for both ports
            self.port1 = mavutil.mavlink_connection('tcp:127.0.0.1:14552')
            self.port2 = mavutil.mavlink_connection('tcp:127.0.0.1:14553')
            self.lock = threading.Lock()
            self.active_commands_port1 = {}
            self.active_commands_port2 = {}
            self.start_listeners()

        def start_listeners(self):
            threading.Thread(target=self.listen_port, args=(self.port1, 'port1'), daemon=True).start()
            threading.Thread(target=self.listen_port, args=(self.port2, 'port2'), daemon=True).start()

        def listen_port(self, port, port_name):
            while True:
                msg = port.recv_match(blocking=True)
                if msg:
                    with self.lock:
                        self.handle_command(msg, port_name)

        def handle_command(self, msg, port_name):
            # Example: Handling SET_MODE commands with priority
            if msg.get_type() == 'SET_MODE':
                if port_name == 'port1':
                    # Port1 has higher priority
                    self.master.set_mode(msg.base_mode, msg.custom_mode)
                    self.active_commands_port1[msg.seq] = msg
                    self.console.notify(f"Port1 (14552) SET_MODE received and executed.")
                elif port_name == 'port2':
                    # Check if there's an active conflicting command from port1
                    if not self.active_commands_port1:
                        self.master.set_mode(msg.base_mode, msg.custom_mode)
                        self.active_commands_port2[msg.seq] = msg
                        self.console.notify(f"Port2 (14553) SET_MODE received and executed.")
                    else:
                        self.console.notify(f"Port2 (14553) SET_MODE ignored due to active command from Port1.")

            # Add handling for other command types as needed
            # For example, handling MAV_CMDs, RC_CHANNELS_OVERRIDE, etc.

        def cmd_priority(self, args):
            """Command to display current priority status"""
            with self.lock:
                port1_cmds = len(self.active_commands_port1)
                port2_cmds = len(self.active_commands_port2)
            self.console.notify(f"Active Commands - Port1 (14552): {port1_cmds}, Port2 (14553): {port2_cmds}")

    def init(mpstate):
        return CommandPriorityModule(mpstate)
""")

def create_custom_module():
    """
    Creates a temporary Python file containing the custom MAVProxy module.
    Returns the path to the module.
    """
    temp_dir = tempfile.gettempdir()
    module_path = os.path.join(temp_dir, "cmd_priority_module.py")
    with open(module_path, "w") as f:
        f.write(CMD_PRIORITY_MODULE_CODE)
    return module_path

def start_mavproxy(module_path):
    """
    Starts MAVProxy with the specified parameters and loads the custom module.
    Returns the subprocess.Popen object.
    """
    cmd = [
        "mavproxy.py",
        f"--master=tcp:{SOURCE_IP}:{SOURCE_PORT}",
        f"--out=tcp:{LOCALHOST}:{DEST_PORT1}",
        f"--out=tcp:{LOCALHOST}:{DEST_PORT2}",
        f"--module={module_path}",
        f"--baudrate={BAUDRATE}",
        f"--aircraft={AIRCRAFT_NAME}",
        "--daemon",
        f"--logfile={LOGFILE}"
    ]

    print(f"Starting MAVProxy with command: {' '.join(cmd)}")
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return process

def shutdown_mavproxy(process):
    """
    Gracefully shuts down the MAVProxy process.
    """
    print("Shutting down MAVProxy...")
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=5)
        print("MAVProxy terminated gracefully.")
    except subprocess.TimeoutExpired:
        print("MAVProxy did not terminate in time. Killing process.")
        process.kill()

def monitor_process(process):
    """
    Monitors the MAVProxy process. Restarts it if it terminates unexpectedly.
    """
    try:
        while True:
            ret = process.poll()
            if ret is not None:
                print("MAVProxy process terminated unexpectedly. Restarting...")
                return False
            time.sleep(5)
    except KeyboardInterrupt:
        shutdown_mavproxy(process)
        sys.exit(0)

def main():
    print("=== MAVLink Forwarder with Command Priority ===")

    # Create the custom MAVProxy module
    module_path = create_custom_module()
    print(f"Custom MAVProxy module created at: {module_path}")

    # Start MAVProxy
    process = start_mavproxy(module_path)

    # Monitor MAVProxy process
    while True:
        alive = monitor_process(process)
        if not alive:
            process = start_mavproxy(module_path)

if __name__ == "__main__":
    main()
