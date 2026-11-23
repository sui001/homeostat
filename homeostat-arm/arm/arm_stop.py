#!/usr/bin/env python3
"""Stop the arm now: Base.Stop() on whatever it is doing. The editor's Stop button runs this.

Deliberately takes no arm lock: it has to work while another script holds it.
    ~/kinova-py310/bin/python arm_stop.py
"""
import os, sys
sys.path.insert(0, os.path.expanduser("~/kinova")); import kenv; kenv.load()
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Session_pb2

t = TCPTransport(); r = RouterClient(t, RouterClient.basicErrorCallback); t.connect("192.168.1.10", 10000)
info = Session_pb2.CreateSessionInfo(); info.username = os.environ["KINOVA_USER"]; info.password = os.environ["KINOVA_PASS"]
info.session_inactivity_timeout = 10000; info.connection_inactivity_timeout = 2000
sm = SessionManager(r); sm.CreateSession(info)
try:
    BaseClient(r).Stop()
    print("Stop sent to the arm.")
finally:
    sm.CloseSession(); r.SetActivationStatus(False)
