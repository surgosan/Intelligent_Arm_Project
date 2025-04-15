from multiprocessing.connection import Client

def send_joint_command(joint_degrees):
    address = ('localhost', 6000)
    conn = Client(address, authkey=b'secret')
    conn.send(joint_degrees)
    response = conn.recv()
    print("Server response:", response)
    conn.close()

if __name__ == '__main__':
    send_joint_command([0, -90, -45, 10])
    def to_home():
        send_joint_command([0, 0, 0, 0])

    # Need to start 1 movement first before actually moving
    print("To Home")
    to_home()
    print("\n\n")

    print("Movement 1")
    send_joint_command([0, 90, -90, 0])
    print("\n\n")

    print("Movement 2")
    send_joint_command([0, -90, 0, 0])
    print("\n\n")

    print("To Home")
    to_home()
    print("\n\n")
