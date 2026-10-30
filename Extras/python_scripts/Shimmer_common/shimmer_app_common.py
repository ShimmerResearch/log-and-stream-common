import sys

from Shimmer_common import shimmer_device


def _port_argument():
    """The COM port given as the first argument, or None. An option such as -v is not a port."""
    if len(sys.argv) > 1 and not sys.argv[1].startswith('-'):
        return sys.argv[1]
    return None


def unittest_argv():
    """sys.argv without the port, for unittest.main(argv=...).

    The suites read their port from the first argument; left in, unittest takes it for the
    name of a test and stops before setUpClass. Options after it (-v, -k pattern, a test name)
    still reach unittest.
    """
    return sys.argv[:1] + sys.argv[2:] if _port_argument() else sys.argv[:]


def get_selected_com_port(dock_ports=True):
    com_port = ''
    if _port_argument() is None:
        if dock_ports:
            options = shimmer_device.serial_ports_shimmer_dock()
        else:
            options = shimmer_device.serial_ports_bluetooth()

        if len(options) > 0:
            user_input = ''
            input_message = "Pick an option:\n"
            for index, item in enumerate(options):
                # input_message += f'{index + 1}) {item}\n'
                input_message += str(index + 1) + ") " + item.name + ", " + item.description + ", " + item.hwid + "\n"
            input_message += "Your choice: "
            # while (not user_input.isnumeric()) or int(user_input) < 1 or int(user_input) > len(options):
            while not user_input or int(user_input) < 1 or int(user_input) > len(options):
                user_input = input(input_message)

            com_port = options[int(user_input) - 1].name
            print("You picked: " + com_port + "\n")
    else:
        com_port = _port_argument()

    return com_port
