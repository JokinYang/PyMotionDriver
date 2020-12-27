import functools
import os
import socket
import threading
import time
from collections import UserDict
from json import JSONEncoder
from socketserver import BaseRequestHandler, ThreadingTCPServer
from typing import SupportsInt
# import datetime

import pandas as pd

from .PyMotionDriver import setup, update, Result


# https://github.com/rlabbe/filterpy
# https://filterpy.readthedocs.io/en/latest/

class ThreadSafeDict(UserDict):
	def __init__(self, *args, **kwargs):
		self.title_without_time = kwargs.pop('title', None)
		self.log_path = kwargs.pop('log_path', "")
		self.lock = threading.RLock()
		self.start_time = time.strftime("%H-%M-%S")
		if not os.path.isdir(self.log_path):
			self.log_path = os.path.dirname(__file__)
		super(ThreadSafeDict, self).__init__(*args, **kwargs)

	# less than
	def __lt__(self, other):
		with self.lock:
			keys = sorted(filter(lambda x: x < other, self.keys()))
			return ThreadSafeDict({k: self.__getitem__(k) for k in keys}, title=self.title_without_time)

	# less equal
	def __le__(self, other):
		with self.lock:
			keys = sorted(filter(lambda x: x <= other, self.keys()))
			return ThreadSafeDict({k: self.__getitem__(k) for k in keys}, title=self.title_without_time)

	# great than
	def __gt__(self, other):
		with self.lock:
			keys = sorted(filter(lambda x: x > other, self.keys()))
			return ThreadSafeDict({k: self.__getitem__(k) for k in keys}, title=self.title_without_time)

	# great equal
	def __ge__(self, other):
		with self.lock:
			keys = sorted(filter(lambda x: x >= other, self.keys()))
			return ThreadSafeDict({k: self.__getitem__(k) for k in keys}, title=self.title_without_time)

	def __setitem__(self, key, value):
		with self.lock:
			if key in self.keys() and self[key] == value:
				return
			return super(ThreadSafeDict, self).__setitem__(key, value)

	def __getitem__(self, item):
		with self.lock:
			if isinstance(item, slice):
				sorted_keys = sorted(self.keys())
				keys = sorted_keys[item]
				return ThreadSafeDict({k: self.__getitem__(k) for k in keys}, title=self.title_without_time)
			else:
				return super(ThreadSafeDict, self).__getitem__(item)

	def cast_with_title(self):
		with self.lock:
			result = []
			title = ['time'] + self.title_without_time
			for k, v in self.sort().items():
				result.append({k_: v_ for k_, v_ in zip(title, [k] + v)})
			return result

	def min_key(self):
		with self.lock:
			if self.keys():
				return min(self.keys())

	def max_key(self):
		with self.lock:
			if self.keys():
				return max(self.keys())

	def sort(self):
		with self.lock:
			if self.min_key():
				return self >= self.min_key()

	def to_csv(self, path=None, *args, **kwargs):
		with self.lock:
			end_time = time.strftime("%H-%M-%S")
			file_path = os.path.join(self.log_path,
									 "MPU6050_{start}_{end}.csv".format(start=self.start_time, end=end_time))
			path = path or file_path
			df = self.to_dataframe()
			df.to_csv(path, *args, **kwargs)

	def to_dataframe(self) -> pd.DataFrame:
		with self.lock:
			l = [[k] + v for k, v in self.sort().items()]
			title = ['time'] + self.title_without_time
			return pd.DataFrame(data=l, columns=title)


class MPU6050SetupError(Exception): pass


def _retry(times=3, interval=.5):
	def decorator(func):
		@functools.wraps(func)
		def wrapper(*args, **kwargs):
			result = None
			for i in range(times):
				try:
					result = func(*args, **kwargs)
					if result:
						break
					else:
						print(f'Retrying ({i}/{times})')
				except Exception:
					print(f'Retrying ({i}/{times})')

				if interval:
					time.sleep(interval)

			return result

		return wrapper

	return decorator


_data_storage = ThreadSafeDict(title=['yaw', 'pitch', 'roll', 'gx', 'gy', 'gz'])


@_retry()
def _read_mpu(storage: ThreadSafeDict, interval: float = 0.02, time_format=None):
	state = setup()
	if state != "Succeed":
		raise MPU6050SetupError("Fail to setup MPU6050,error msg:{}".format(state))
	while True:
		s = time.time()
		r: Result = update()
		# current_time = datetime.datetime.now().strftime(time_format or '%Y-%m-%d %H:%M:%S.%f')
		current_time = time.time_ns()
		# title: ['yaw', 'pitch', 'roll', 'gx', 'gy', 'gz']
		storage[current_time] = [r.ypr.x, r.ypr.y, r.ypr.z, r.gyro.x, r.gyro.y, r.gyro.z]
		e = time.time()
		spend = e - s
		if spend < interval:
			time.sleep(interval - spend)


class MPU6050Handler(BaseRequestHandler):

	def setup(self) -> None:
		pass

	def handle(self) -> None:
		global _data_storage
		conn: socket.socket = self.request
		while True:
			msg = str(conn.recv(1024), encoding='utf-8')
			if not msg:
				continue
			if msg == 'a':
				pass

			v = _data_storage[-1:]
			if not v:
				conn.send(b'')
				continue
			yprt = v.cast_with_title()
			content = JSONEncoder().encode(yprt)
			conn.send(bytes(content, encoding='utf-8'))

	def finish(self):
		self.request.close()


def run_forwarder_server(ip: str, port: SupportsInt) -> SupportsInt:
	try:
		server_address = (ip, port)
		tcp_server = ThreadingTCPServer(server_address, MPU6050Handler)
		tcp_server.serve_forever()
	except OSError as e:
		if e.errno == 98:
			port += 1
			return run_forwarder_server(ip, port)
	return port


def run_read_thread():
	global _data_storage
	read_thread = threading.Thread(target=_read_mpu, args=(_data_storage,), daemon=False)
	read_thread.start()


# TODO add kalman filter
# TODO add command line to setup
run_read_thread()
run_forwarder_server('0.0.0.0', 916)
