import socket
import thread
import time
# TODO: Client id assignment - To get clients from id (Process id)
# TODO: Client user type assignmen - To check user priviledges

class SocketServer(socket.socket):
    clients = []
    def __init__(self):
        socket.socket.__init__(self)
        #To silence- address occupied!!
        self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.bind(('0.0.0.0', 9090))
        self.listen(5)

    def run(self, port):
        self.port = port
        print "Server started"
        try:
            self.accept_clients()
        except Exception as ex:
            print ex
        finally:
            print "Server closed"
            for client in self.clients:
                client.close()
            self.close()

    def accept_clients(self):
        while 1:
            (clientsocket, address) = self.accept()
            # print self.clients

            #Adding client to clients list
            self.clients.append(clientsocket) # FIXME
            #Client Connected
            self.onopen(clientsocket)
            #Receiving data from client
            thread.start_new_thread(self.recieve, (clientsocket,))

    def recieve(self, client):
        # CHANGED: time_step = 0.1
        time_step = 0.01
        timeout = 100
        c_time = 0
        while c_time < timeout:
            try:
                data = client.recv(1024)
                if data == '':
                    break
                #Message Received
                self.onmessage(client, data.split("\n")[0])
            except:
                pass
            timeout += time_step
            time.sleep(time_step)
        #Removing client from clients list
        self.clients.remove(client)
        #Client Disconnected
        self.onclose(client)
        #Closing connection with client
        client.close()
        #Closing thread
        thread.exit()

    def send_to_specific_client(self, client, message):
        # Sending message only to a client
        for c in self.clients:
            if c == client:
                c.send(message)
    def send_to_all_except(self, client, message):
        # Sending message to all except a client
        for c in self.clients:
            if c != client:
                c.send(message)

    def broadcast(self, message):
        #Sending message to all clients
        for client in self.clients:
            client.send(message)

    def onopen(self, client):
        pass

    def onmessage(self, client, message):
        pass

    def onclose(self, client):
        pass



class BasicChatServer(SocketServer):
    def __init__(self, on_message):
        self.on_message = on_message
        self.client_list = []
        SocketServer.__init__(self)

    def onmessage(self, client, message):

        self.on_message(client, message, self.port)
        # print "Received From Client: " + str(message)
        #Sending message to all clients
        # self.send_to_all_except(client, message)
        # self.broadcast(message)

    def onopen(self, client):
        print "Client Connected"

    def onclose(self, client):
        print "Client Disconnected"
