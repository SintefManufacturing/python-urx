#! /usr/bin/env python

import logging


# Controller Settings
CONTROLLER_PORTS = [0, 1]
CONTROLLER_VOLTAGE = [
    0,  # 0-5V
    2,  # 0-10V
]

# Tool Settings
TOOL_PORTS = [2, 3]
TOOL_VOLTAGE = [
    0,  # 0-5V
    1,  # 0-10V
    2,  # 4-20mA
]

OUTPUT_DOMAIN_VOLTAGE = [
    0,  # 4-20mA
    1,  # 0-10V
]


class URScript(object):

    def __init__(self):
        self.logger = logging.getLogger(u"urscript")
        # The header is code that is before and outside the myProg() method
        self.header = ""
        # The program is code inside the myProg() method
        self.program = ""

    def __call__(self):
        if(self.program == ""):
            self.logger.debug(u"urscript program is empty")
            return ""

        # Construct the program
        myprog = """def myProg():{}\nend""".format(self.program)

        # Construct the full script
        script = ""
        if self.header:
            script = "{}\n\n".format(self.header)
        script = "{}{}".format(script, myprog)
        return script

    def reset(self):
        self.header = ""
        self.program = ""

    def add_header_to_program(self, header_line):
        self.header = "{}\n{}".format(self.header, header_line)

    def add_line_to_program(self, new_line):
        self.program = "{}\n\t{}".format(self.program, new_line)

    def _constrain_unsigned_char(self, value):
        """
        Ensure that unsigned char values are constrained
        to between 0 and 255.
        """
        assert(isinstance(value, int))
        if value < 0:
            value = 0
        elif value > 255:
            value = 255
        return value

    def _set_analog_inputrange(self, port, vrange):
        if port in CONTROLLER_PORTS:
            assert(vrange in CONTROLLER_VOLTAGE)
        elif port in TOOL_PORTS:
            assert(vrange in TOOL_VOLTAGE)
        msg = "set_analog_inputrange({},{})".format(port, vrange)
        self.add_line_to_program(msg)

    def _set_analog_output(self, input_id, signal_level):
        assert(input_id in [0, 1])
        assert(signal_level in [0, 1])
        msg = "set_analog_output({}, {})".format(input_id, signal_level)
        self.add_line_to_program(msg)

    def _set_analog_outputdomain(self, port, domain):
        assert(domain in OUTPUT_DOMAIN_VOLTAGE)
        msg = "set_analog_outputdomain({},{})".format(port, domain)
        self.add_line_to_program(msg)

    def _set_payload(self, mass, cog=None):
        msg = "set_payload({}".format(mass)
        if cog:
            assert(len(cog) == 3)
            msg = "{},{}".format(msg, cog)
        msg = "{})".format(msg)
        self.add_line_to_program(msg)

    def _set_runstate_outputs(self, outputs=None):
        if not outputs:
            outputs = []
        msg = "set_runstate_outputs({})".format(outputs)
        self.add_line_to_program(msg)

    def _set_tool_voltage(self, voltage):
        assert(voltage in [0, 12, 24])
        msg = "set_tool_voltage({})".format(voltage)
        self.add_line_to_program(msg)

    def _sleep(self, value):
        msg = "sleep({})".format(value)
        self.add_line_to_program(msg)

    def _socket_close(self, socket_name):
        msg = "socket_close(\"{}\")".format(socket_name)
        self.add_line_to_program(msg)

    def _socket_get_var(self, var, socket_name):
        msg = "socket_get_var(\"{}\",\"{}\")".format(var, socket_name)
        self.add_line_to_program(msg)
        self._sync()

    def _socket_open(self, socket_host, socket_port, socket_name):
        msg = "socket_open(\"{}\",{},\"{}\")".format(socket_host,
                                                     socket_port,
                                                     socket_name)
        self.add_line_to_program(msg)

    def _socket_read_byte_list(self, nbytes, socket_name):
        msg = "global var_value = socket_read_byte_list({},\"{}\")".format(nbytes, socket_name)  # noqa
        self.add_line_to_program(msg)
        self._sync()

    def _socket_send_string(self, message, socket_name):
        msg = "socket_send_string(\"{}\",\"{}\")".format(message, socket_name)  # noqa
        self.add_line_to_program(msg)
        self._sync()

    def _socket_set_var(self, var, value, socket_name):
        msg = "socket_set_var(\"{}\",{},\"{}\")".format(var, value, socket_name)  # noqa
        self.add_line_to_program(msg)
        self._sync()

    def _sync(self):
        msg = "sync()"
        self.add_line_to_program(msg)
