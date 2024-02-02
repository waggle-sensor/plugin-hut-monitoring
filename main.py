import argparse
import logging
import time
from multiprocessing import Process, Lock, Event, Manager

from serial import Serial
import schedule

from waggle.plugin import Plugin

# the list is ordered by the sensor output
# do not reorder them
topic_ordered_list = [
    "env.temperature.hut",
    "env.humidity.hut",
    "env.roll.hut",
    "env.pitch.hut",
    "env.yaw.hut",
]

meta_table = {
    "env.temperature.hut": {"unit": "C", "sensor": "AHT21"},
    "env.humidity.hut": {"unit": "%", "sensor": "AHT21"},
    "env.roll.hut": {"unit": "degree", "sensor": "MPU6050"},
    "env.pitch.hut": {"unit": "degree", "sensor": "MPU6050"},
    "env.yaw.hut": {"unit": "degree", "sensor": "MPU6050"},
}


class HutMonitoring(Process):
    def __init__(self, args):
        super(HutMonitoring, self).__init__()
        self.device = args.device
        self.baudrate = args.baudrate
        self.mutex = Lock()
        self.halt = Event()
        self.m = Manager()
        self.cache = self.m.dict()

    def __enter__(self):
        if not self.is_alive():
            self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.halt.set()
        self.join()
        self.close()

    def run(self):
        with Serial(self.device, baudrate=self.baudrate) as serial:
            while not self.halt.is_set():
                line = serial.readline().decode().strip()
                if line == "":
                    continue
                self.mutex.acquire()
                self.cache.update({
                    "data": line,
                    "timestamp": time.time_ns(),
                })
                self.mutex.release()
                logging.debug(f'read from serial: {line}')

    def read(self):
        c = ""
        t = 0
        self.mutex.acquire()
        if "data" in self.cache:
            c = self.cache["data"]
        if "timestmap" in self.cache:
            t = self.cache["timestamp"]
        self.mutex.release()
        logging.debug(f'read cache {c} {t}')
        return c, t
    
    def read_parsed(self):
        c, t = self.read()
        if c == "":
            return {"error": "no data found"}, t

        # temp, humidity,roll,pitch,yaw
        # C, %, degree, degree, degree
        # 10.57,53.43,145.41,0.03,0.17
        sp = c.strip().split(",")
        if len(sp) != 5:
            return {"error": f'format error: must have 5 values, but got {c}'}, t
        v = {}
        for topic, value in zip(topic_ordered_list, sp):
            try:
                converted = float(value)
                v[topic] = converted
            except ValueError as ex:
                return {"error": f'{ex}'}, t
        return v, t


def job(h, plugin):
    v_dict, timestamp = h.read_parsed()
    if "error" in v_dict:
        logging.error(f'{v_dict["error"]}')
    else:
        for k, v in v_dict.items():
            if k in meta_table:
                meta = meta_table[k]
            else:
                meta = {}
            logging.debug(f'publishing {k}, {v}, {meta}')
            plugin.publish(k, v, timestamp=timestamp, meta=meta)
        logging.info(f'published {len(v_dict.keys())} measurements')


def main(args):
    if args.interval < 3:
        logging.error(f'{args.interval} must be greater than 2 seconds because the sensor output is around 2 Hz')
        args.interval = 3
        logging.error(f'interval is set to {args.interval} seconds')

    with HutMonitoring(args) as h, Plugin() as plugin:
        schedule.every(args.interval).seconds.do(job, h=h, plugin=plugin)

        while len(schedule.get_jobs()) > 0:
            schedule.run_pending()
            logging.debug("sleeping")
            time.sleep(1)
        logging.info("No jobs are available. They might have been removed due to an error")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--debug", dest="debug",
        action="store_true",
        help="Enable debugging")
    parser.add_argument(
        "--device", dest="device",
        action="store", type=str,
        help="Path to serial device")
    parser.add_argument(
        "--baud-rate", dest="baudrate",
        action="store", default=19200, type=int,
        help="Baudrate of serial device. Default: 19200")
    parser.add_argument(
        "--publish-interval", dest="interval",
        action="store", default=30, type=int,
        help="Publish interval in seconds")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format='%(asctime)s %(levelname)s: %(message)s',
        datefmt='%Y/%m/%d %H:%M:%S')

    exit(main(args))
