import SocketServer
import subprocess
import os
import re
import logging
import sys

### takes a variable - slugsin file

logging.basicConfig(level=logging.INFO,
                    format='%(name)s: %(message)s',
                    )

class EchoRequestHandler(SocketServer.BaseRequestHandler):

    def __init__(self, request, client_address, server):
        self.logger = logging.getLogger('EchoRequestHandler')
        self.logger.debug('__init__')
        SocketServer.BaseRequestHandler.__init__(self, request, client_address, server)
        return

    def setup(self):
        self.logger.debug('setup')
        return SocketServer.BaseRequestHandler.setup(self)

    def handle(self):
        self.logger.debug('handle')

        # Echo the back to the client
        data = self.request.recv(1024)
        self.logger.debug('recv()->"%s"', data)
        data = data[0:len(data) - 1]
        if data == "inputs":
            result = ",".join(self.server.getInputs())
        elif data == "outputs":
            result = ",".join(self.server.getOuputs())
        elif data == "state":
            result = self.server.getState()
        elif data.startswith("trans"):
            result = self.server.getTrans(data.lstrip("trans:"))
        else:
            result = "What do you mean by {}".format(data)

        self.request.sendall(result)
        return

    def finish(self):
        self.logger.debug('finish')
        return SocketServer.BaseRequestHandler.finish(self)

class EchoServer(SocketServer.TCPServer):

    def __init__(self, server_address, slugsin_file, handler_class=EchoRequestHandler):
        self.logger = logging.getLogger('EchoServer')
        self.logger.debug('__init__')
        SocketServer.TCPServer.__init__(self, server_address, handler_class)
        print "I am created again!"
        self.startSLUGS(slugsin_file)
        #self.startSLUGS(r"/cygdrive/c/Users/Catherine/LTLMoP/src/examples/multiRobot/three_robots_three_items/three_robots_three_items.slugsin")
        return

    def server_activate(self):
        self.logger.debug('server_activate')
        SocketServer.TCPServer.server_activate(self)
        return

    def serve_forever(self):
        self.logger.debug('waiting for request')
        self.logger.info('Handling requests, press <Ctrl-C> to quit')
        while True:
            self.handle_request()
        return

    def handle_request(self):
        self.logger.debug('handle_request')
        return SocketServer.TCPServer.handle_request(self)

    def verify_request(self, request, client_address):
        self.logger.debug('verify_request(%s, %s)', request, client_address)
        return SocketServer.TCPServer.verify_request(self, request, client_address)

    def process_request(self, request, client_address):
        self.logger.debug('process_request(%s, %s)', request, client_address)
        return SocketServer.TCPServer.process_request(self, request, client_address)

    def server_close(self):
        self.logger.debug('server_close')
        return SocketServer.TCPServer.server_close(self)

    def finish_request(self, request, client_address):
        self.logger.debug('finish_request(%s, %s)', request, client_address)
        return SocketServer.TCPServer.finish_request(self, request, client_address)

    def close_request(self, request_address):
        self.logger.debug('close_request(%s)', request_address)
        return SocketServer.TCPServer.close_request(self, request_address)

    ###### SLUGS FUNCTIONS ######
    def startSLUGS(self, filename):
        slugs_path = os.path.join("src", "slugs")
        self.slugsProcess = subprocess.Popen(slugs_path + " --interactiveStrategy " + filename, shell=True, bufsize=1048000, stdin=subprocess.PIPE, stdout=subprocess.PIPE)

    def getInputs(self):
        # Get input APs
        self.slugsProcess.stdin.write("XPRINTINPUTS\n")
        self.slugsProcess.stdin.flush()
        self.slugsProcess.stdout.readline()  # Skip the prompt
        lastLine = " "
        self.inputAPs = []
        while (lastLine != ""):
            lastLine = self.slugsProcess.stdout.readline().strip()
            if lastLine != "":
                self.inputAPs.append(lastLine)

        return self.inputAPs

    def getOuputs(self):
        # Get output APs
        self.slugsProcess.stdin.write("XPRINTOUTPUTS\n")
        self.slugsProcess.stdin.flush()
        self.slugsProcess.stdout.readline()  # Skip the prompt
        lastLine = " "
        self.outputAPs = []
        while (lastLine != ""):
            lastLine = self.slugsProcess.stdout.readline().strip()
            if lastLine != "":
                lastLine = re.sub(r'^bit(\d+)$', r'region_b\1', lastLine)
                self.outputAPs.append(lastLine)
        return self.outputAPs

    def getState(self):
        self.slugsProcess.stdin.write("XGETINIT\n")  # XCOMPLETEINIT\n" + initInputsOutputs)#
        self.slugsProcess.stdin.flush()
        self.slugsProcess.stdout.readline()  # Skip the prompt
        currentState = self.slugsProcess.stdout.readline().strip()
        return currentState

    def getTrans(self, nextInput):
        self.slugsProcess.stdin.write("XMAKETRANS\n" + nextInput)
        self.slugsProcess.stdin.flush()
        self.slugsProcess.stdout.readline()  # Skip the prompt
        nextLine = self.slugsProcess.stdout.readline().strip()
        return nextLine

if __name__ == "__main__":
    import threading
    import argparse
    logger = logging.getLogger('Main')
    parser = argparse.ArgumentParser(description="SLUGS Server.")
    parser.add_argument('slugsin_file', type=str, help='slugsin file')
    parser.add_argument('--port', type=int, help='port number')
    args = parser.parse_args()

    try:
        if args.port:
            address = ('localhost', args.port)  # let the kernel give us a port
        else:
            address = ('localhost', 9999)  # let the kernel give us a port
        server = EchoServer(address, args.slugsin_file, EchoRequestHandler)
        ip, port = server.server_address  # find out what port we were given

        t = threading.Thread(target=server.serve_forever)
        t.setDaemon(True)  # don't hang on exit
        logger.info("Starting slugs server")
        t.start()
        raw_input()
    finally:

        logger.info("Stopping slugs server...")
        server.socket.close()
        t.join()

#python slugsServer.py --port 9999 ../../examples/multiRobot_decent/one_robot_three_items_old_eight_regions/rob1/one_robot_three_items.slugsin
#python slugsServer.py --port 9989 ../../examples/multiRobot_decent/one_robot_three_items_old_eight_regions/rob2/one_robot_three_items.slugsin
#python slugsServer.py --port 9979 ../../examples/multiRobot_decent/one_robot_three_items_old_eight_regions/rob3/one_robot_three_items.slugsin
#python __MATLABMultiRobotServer.py --region_file_directory ../../../../examples/multiRobot_decent/one_robot_three_items_old_eight_regions/rob1/  --region_filename one_robot_three_items --no_of_robots 3
#python specEditor.py examples/multiRobot_decent/one_robot_three_items_old_eight_regions/rob1/one_robot_three_items
#python specEditor.py examples/multiRobot_decent/one_robot_three_items_old_eight_regions/rob2/one_robot_three_items
#python specEditor.py examples/multiRobot_decent/one_robot_three_items_old_eight_regions/rob3/one_robot_three_items
