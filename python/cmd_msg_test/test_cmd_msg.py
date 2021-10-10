"""


Compile QEMU Version 5.1.0 or newer. 5.1.0 is when AVR support was introduced.

.. code-block:: console

    $ wget https://download.qemu.org/qemu-6.1.0.tar.xz
    $ tar xvJf qemu-6.1.0.tar.xz
    $ cd qemu-6.1.0
    $ ./configure --target-list="avr-softmmu"
    $ make -j $(($(nproc)*4))

Change directory to this file's parent directory and run using unittest

.. code-block:: console

    $ cd python/cmd_msg_test/
    $ python -u -m unittest discover -v
    test_connect (test_cmd_msg.TestSerial) ... qemu-system-avr: -chardev socket,id=serial_port,path=/tmp/tmpuuq3oqvj/socket,server=on: info: QEMU waiting for connection on: disconnected:unix:/tmp/tmpuuq3oqvj/socket,server=on
    reading message from arduino
    b''
    b''
    qemu-system-avr: terminating on signal 2 from pid 90395 (python)
    ok

    ----------------------------------------------------------------------
    Ran 1 test in 4.601s

    OK
"""
import os
import sys
import signal
import pathlib
import tempfile
import unittest
import subprocess
import contextlib
import dataclasses

# top level directory in this git repo is three levels up
REPO_ROOT = pathlib.Path(__file__).parents[2].resolve()


@contextlib.contextmanager
def start_qemu(bios):
    with tempfile.TemporaryDirectory() as tempdir:
        socket_path = pathlib.Path(tempdir, "socket")
        qemu_cmd = [
            "qemu-system-avr",
            "-mon",
            "chardev=none",
            "-chardev",
            f"null,id=none",
            "-serial",
            "chardev:serial_port",
            "-chardev",
            f"socket,id=serial_port,path={socket_path},server=on",
            "-nographic",
            "-machine",
            "arduino-uno",
            "-cpu",
            "avr6-avr-cpu",
            "-bios",
            bios,
        ]
        qemu_proc = subprocess.Popen(qemu_cmd, start_new_session=True)
        serial_port_path = pathlib.Path(tempdir, "ttyACM0")
        socat_cmd = [
            "socat",
            f"PTY,link={serial_port_path},rawer,wait-slave",
            f"UNIX:{socket_path}",
        ]
        socat_proc = subprocess.Popen(socat_cmd, start_new_session=True)
        try:
            yield str(serial_port_path)
        finally:
            # Kill the whole process group (for problematic processes like qemu)
            os.killpg(qemu_proc.pid, signal.SIGINT)
            os.killpg(socat_proc.pid, signal.SIGINT)
        qemu_proc.wait()
        socat_proc.wait()


class TestSerial(unittest.TestCase):
    def test_connect(self):
        with start_qemu(
            str(REPO_ROOT.joinpath("build", "serial_cmd_test.ino.elf"))
        ) as serial_port:
            os.environ["SERIAL_PORT"] = serial_port
            subprocess.check_call([sys.executable, "main.py"])
