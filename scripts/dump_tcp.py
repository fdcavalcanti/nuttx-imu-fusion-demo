from communicate import connect_to_surface

if __name__ == "__main__":
    socket = connect_to_surface("192.168.0.3", 5000)
    while True:
        try:
            data = socket.recv(1024)
            print(data.decode(), end="")
        except KeyboardInterrupt:
            socket.close()
