#!/usr/bin/env python
"""
vel = 122
acc = 124
"""

import pickle
import sys
import threading
import time

import serial

port = 'COM30'
velocity = 1000.0
acceleration = 5000.0
hb_counter = 0
preample_filename = 'preamble.p'

velocity_scalar = 8279.2205
acc_scalar = 2.45138

if len(sys.argv) > 1:
    velocity = float(sys.argv[1])

if len(sys.argv) > 2:
    acceleration = float(sys.argv[2])
    

def checksum(msg):
    return ((0xFF - sum(msg)) + 1) & 0x7F


class Heartbeat:
    """
    First send 0, 0[header], then send 3, b0, b1 every 1/2 second
    """
    def __init__(self):
        self.b0 = -2  # hack to start at 0
        self.b1 = 0
    
    def header(self):
        msg = [176, 2, 0, 0]
        msg.append(checksum(msg))
        return bytes(msg)
    
    def reset(self, b0=-2, b1=0):
        self.b0 = b0
        self.b1 = b1
    
    def __next__(self):
        self.b0 += 2
        if self.b0 > 127:
            self.b0 = 0
            self.b1 += 1
            if self.b1 > 127:
                raise NotImplementedError()
        msg = [176, 3, 3, self.b0, self.b1]
        msg.append(checksum(msg))
        return bytes(msg)


def test_heartbeat():
    hb = Heartbeat()
    print(next(hb))
    print(next(hb))


def read_message(s, timeout=3.0):
    if timeout < 0:
        raise IOError("read_message timed out")
    t0 = time.time()
    # read packet id
    # if \x00 try again
    pid = s.read(1)
    if len(pid) == 0:
        return read_message(s, timeout - (time.time() - t0))
    if ord(pid) == 0:
        return read_message(s, timeout - (time.time() - t0))
    dlen = s.read(1)
    data = s.read(ord(dlen))
    cs = s.read(1)
    return pid + dlen + data + cs
    #return bytes(pid, dlen + data + [cs, ])
    

def send_preamble(s):
    s.baudrate = 9600
    time.sleep(0.2)
    s.write(bytes([240, 3, 9, 48, 0, 84]))
    rmsg = read_message(s)
    if rmsg != bytes([240, 3, 9, 48, 0, 84]):
        raise IOError("Invalid response: %s" % (rmsg, ))
    s.baudrate = 230400
    time.sleep(0.2)


def send_saved_preamble(s):
    """
    Set baud rate: 9600
    W: [240, 3, 9, 48, 0, 84]
    R: [240, 3, 9, 48, 0, 84]
    Set baud rate: 230400
    Sending velocity: 56.0
    """
    with open(preample_filename, 'rb') as f:
        pd = pickle.load(f)
    hb = Heartbeat()
    last_sent_hb = None
    for p in pd:
        (direction, data) = p
        if (direction == 1) and isinstance(data, int):
            # set serial baud rate
            print("Set baud rate: %s" % (data, ))
            s.baudrate = data
            #if data == 9600:
            #    s.send_break(0.04)
            # TODO flush?
            time.sleep(0.20)
            continue
        msg = bytes([data[0], data[1]] + data[2] + [data[3], ])
        if direction == 1:
            # write packet
            print("W: %s" % (list(msg), ))
            s.write(msg)
            if data[0] == 176:
                if data[2][0] == 3:
                    hb.reset(data[2][1], data[2][2])
                    last_sent_hb = time.time()
        else:
            # wait for packet
            rmsg = read_message(s)
            print("R: %s" % (list(rmsg), ))
            if rmsg != msg:
                raise IOError(
                    "Received Unexpected message: %s != %s" %
                    (list(rmsg), list(msg)))
    return hb, last_sent_hb


def pack_velocity(velocity):
    if velocity < 0.01 or velocity > 4000.:
        raise ValueError("Velocity out of range: %s" % velocity)
    bmsg = [128, 7, 1, 122]
    # add velocity 4 bytes
    # lowest bit is 0
    # for first message bit 1 is 0, for second bit 1 is 1
    # remaining bits are spread across 4 bytes (1st byte includes lowest bits)
    # no top bit (bit7) in any byte is used
    # so bytes are:
    # 1: [ -,  4,  3,  2,  1,  0,  _,  _]
    # 2: [ -, 11, 10,  9,  8,  7,  6,  5]
    # 3: [ -, 18, 17, 16, 15, 14, 13, 12]
    # 4: [ -, 25, 24, 23, 22, 21, 20, 19]
    # max is 4000 rpm, scaled by * 8279.3032
    # compute checksum
    sv = int(velocity * velocity_scalar)
    bmsg.append((sv & 0x1F) << 2)
    bmsg.append((sv >> 5) & 0x7F)
    bmsg.append((sv >> 12) & 0x7F)
    bmsg.append((sv >> 19) & 0x7F)
    bmsg.append(0)
    msg = bmsg[:]
    msg.append(checksum(msg))
    bmsg[4] |= 0b10
    bmsg.append(checksum(bmsg))
    msg += bmsg
    return msg

    
def pack_acceleration(acc):
    if acc < 0.01 or acc > 100000.:
        raise ValueError("Acceleration out of range: %s" % acc)
    bmsg = [128, 7, 1, 124]
    sv = int(acc * acc_scalar)
    bmsg.append((sv & 0x1F) << 2)
    bmsg.append((sv >> 5) & 0x7F)
    bmsg.append((sv >> 12) & 0x7F)
    bmsg.append((sv >> 19) & 0x7F)
    bmsg.append(0)
    msg = bmsg[:]
    msg.append(checksum(msg))
    bmsg[4] |= 0b10
    bmsg.append(checksum(bmsg))
    msg += bmsg
    return msg


def test_send_old():
    s = serial.Serial(port, 9600)
    s.write([0xb0, 0x02, 0x00, 0x00, 0x4e])  # header
    s.write(bytes([0xf0, 0x02, 0x00, 0x00, 0x0e]))  # heartbeat
    s.write(bytes([0xd0, 0x00, 0x30]))  # wakeup?
    s.write(bytes([0x80, 0x02, 0x05, 0x00, 0x79]))  # baudrate get
    s.write(bytes([0xf0, 0x03, 0x09, 0x18, 0x00, 0x6c]))  # baudrate set
    s.baudrate = 115200
    s.write(bytes([0xb0, 0x00, 0x30]))
    print("Sending velocity: %s" % velocity)
    try:
        msg = pack_velocity(velocity)
        s.write(msg)
        time.sleep(0.1)
        if s.inWaiting():
            print("Read: %s" % s.read(s.inWaiting()))
    except ValueError:
        pass
    s.close()

    
def test_send():
    s = serial.Serial(None, 9600, timeout=1.0)
    s.rts = False
    s.dtr = False
    s.port = port
    s.open()
    #with serial.Serial(port, 9600, timeout=1.0) as s:
    #vmsg =  bytes([
    #    128, 7, 1, 122, 24, 123, 89, 23, 0, 123,
    #    128, 7, 1, 122, 26, 123, 89, 23, 0, 121])
    with s:
        # hb, lhb = send_preamble(s)
        send_preamble(s)
        print("Sending velocity: %s" % velocity)
        msg = pack_velocity(velocity)
        #msg = vmsg
        print("W: %s" % (list(msg), ))
        s.write(msg)
        msg = pack_acceleration(acceleration)
        print("W: %s" % (list(msg), ))
        s.write(msg)
    s.close()


def connect(port):
    #s = serial.Serial(None, 9600, timeout=1.0)
    #s.rts = False
    #s.dtr = False
    #s.port = port
    #s.open()
    s = serial.Serial(port, 9600, timeout=1.0)
    send_preamble(s)
    return s


def send_velocity(conn, vel):
    print("Sending velocity: %s" % vel)
    msg = pack_velocity(vel)
    print("W: %s" % (list(msg), ))
    conn.write(msg)


def send_acceleration(conn, acc):
    print("Sending acceleration: %s" % acc)
    msg = pack_acceleration(acc)
    print("W: %s" % (list(msg), ))
    conn.write(msg)


class Clearpath(object):
    def __init__(self, port):
        self._lock = threading.Lock()
        self._thread = None
        self._port = port

    def _thread_func(self):
        self.conn = connect(port)
        self.hb = Heartbeat()
        with self._lock:
            #print("Write header")
            self.conn.write(self.hb.header())
        while True:
            with self._lock:
                if self.conn is None:
                    break
                msg = next(self.hb)
                self.conn.write(msg)
                #print("Write next hb: %s" % (list(msg), ))
            time.sleep(0.5)
        del self.hb
    
    def _start_thread(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._thread_func)
        self._thread.start()
        # wait for connection
        while not hasattr(self, 'conn'):
            time.sleep(0.01)
    
    def _stop_thread(self):
        if self._thread is None:
            return
        with self._lock:
            self.conn = None
        self._thread.join()
        self._thread = None
    
    def send_message(self, msg):
        if self._thread is None:
            self._start_thread()
        with self._lock:
            self.conn.write(msg)
    
    def send_velocity(self, vel):
        msg = pack_velocity(vel)
        self.send_message(msg)
        
    def send_acceleration(self, acc):
        msg = pack_velocity(acc)
        self.send_message(msg)
    
    def __del__(self):
        self._stop_thread()


if __name__ == '__main__':
    #test_heartbeat()
    #test_send()
    
    conn = connect(port)
    hb = Heartbeat()
    vels = [50, 100, 200, 400, 800, 1600, 3200]
    
    with conn:
        #conn.write(hb.header())
        #conn.write(next(hb))
        send_acceleration(conn, acceleration)
        for v in vels:
            send_velocity(conn, v)
            print("Set velocity: %s" % v)
            time.sleep(5.0)
        #send_acceleration(conn, acceleration / 2.)
        #print("Waiting, sending hb")
        #for _ in range(10):
        #    time.sleep(0.5)
        #    conn.write(next(hb))
        #send_velocity(conn, velocity)
        #send_acceleration(conn, acceleration)
    conn.close()
    
    #cp = Clearpath(port)