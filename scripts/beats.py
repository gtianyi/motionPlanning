#!/usr/bin/env python3

from subprocess import TimeoutExpired, PIPE, run
from builtins import filter
from os import path
from threading import Thread
import paramiko
from tqdm import tqdm
from paramiko import SSHException
from queue import Queue, Empty
import configuration_generator
import configuration_parser
import datetime
import json
import slack_notification

__author__ = 'Bence Cserna (bence@cserna.net)'

HOSTS = ['ai' + str(i) + '.cs.unh.edu' for i in [1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15]]


class Experiment:
    def __init__(self, configuration, command):
        self.configuration = configuration
        self.command = command
        self.raw_result = None
        self.result = None
        self.error = None


def process_log(log_lines):
    """Process output of OMPL/motionPlanner and return a dictionary with the relevant values."""

    log_lines = [line.strip() for line in log_lines]

    save_header = False
    save_results = False

    headers = []
    attribute_sets = []
    for line in log_lines:
        # Skip empty lines 
        if not line:
            continue

        if save_results:
            attribute_sets.append(list(filter(bool, line.split(';'))))

        if save_header and 'runs' in line:
            save_header = False
            save_results = True

        if save_header:
            headers.append(line)

        if 'properties for each run' in line:
            save_header = True

    result = {}
    for attribute_set in attribute_sets:
        if len(attribute_set) != len(headers):
            continue

        for header, attribute in zip(headers, attribute_set):
            attribute = attribute.strip()
            type = header.split()[-1]
            if type == 'BOOLEAN':
                typed_attribte = attribute in ['1']
            elif type == 'INTEGER' or type == 'ENUM':
                typed_attribte = int(attribute)
            elif type == 'REAL':
                typed_attribte = float(attribute)
            else:
                raise ValueError('Unknown header type')

            result[' '.join(header.split()[:-1])] = typed_attribte

    return result


def assemble_experiments(configurations):
    """Set up executable commands for the given list of experiments."""

    command_template = 'export LD_LIBRARY_PATH=/home/aifs1/gu/lib:$LD_LIBRARY_PATH &&' \
                       'cd /home/aifs1/gu/gopath/src/github.com/gu/motionPlanning/build_release/ &&' \
                       'printf "{configuration}" | ' \
                       '/home/aifs1/gu/gopath/src/github.com/gu/motionPlanning/build_release/MotionPlanning'

    return [Experiment(configuration_parser.parse_configuration(configuration),
                       command_template.format(configuration=configuration))
            for configuration in configurations]


class Worker(Thread):
    def __init__(self, hostname, job_queue, result_queue, progress_bar):
        super(Worker, self).__init__()
        self.progress_bar = progress_bar
        self.processed_experiment_queue = result_queue
        self.experiment_queue = job_queue
        self.hostname = hostname

    def run(self):
        client = spawn_ssh_client(self.hostname)
        while True:
            try:
                experiment = self.experiment_queue.get(block=False)
                stdin, stdout, stderr = client.exec_command(experiment.command)
                result = {'stdout': stdout.readlines(), 'stderr': stderr.readlines()}
                experiment.raw_result = result
                experiment.error = result['stderr']
                experiment.result = process_log("".join(result['stdout']).splitlines())
                self.processed_experiment_queue.put(experiment)
                self.progress_bar.update()
                self.experiment_queue.task_done()

            except Empty:
                break


def spawn_ssh_client(hostname):
    key = paramiko.RSAKey.from_private_key_file(path.expanduser("~/.ssh/id_rsa"))
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(hostname=hostname, pkey=key)
    return client


def execute_configuration(configuration, timeout=100000):
    command = ['../cmake-build-debug/MotionPlanning']

    try:
        completed_process = run(command, input=configuration.encode('utf-8'), stdout=PIPE, timeout=timeout)
    except TimeoutExpired:
        return [{'configuration': configuration, 'success': False, 'errorMessage': 'timeout'}
                for configuration in configuration]
    except Exception as e:
        return [{'configuration': configuration, 'success': False, 'errorMessage': 'unknown error ::' + str(e)}
                for configuration in configuration]

    # Create error configurations if the execution failed
    if completed_process.returncode != 0:
        message = completed_process.stdout.decode('utf-8')
        return [{'configuration': configuration, 'success': False, 'errorMessage': 'execution failed ::' + message}
                for configuration in configuration]

    raw_output = completed_process.stdout.decode('utf-8').splitlines()

    results = process_log(raw_output)
    return results


def create_workers_for_hosts(hosts, command_queue, result_queue, progress_bar):
    return [Worker(hostname=host, job_queue=command_queue,
                   result_queue=result_queue, progress_bar=progress_bar) for host in hosts]


def main():
    command_queue = Queue()
    result_queue = Queue()
    experiments = assemble_experiments(configuration_generator.generate_configurations())

    for item in experiments:
        command_queue.put(item)

    progress_bar = tqdm(total=command_queue.qsize())
    workers = create_workers_for_hosts(HOSTS, command_queue, result_queue, progress_bar)

    slack_notification.start_experiment_notification(len(experiments))

    # Start workers
    for worker in workers:
        worker.start()

    # Wait for workers
    for worker in workers:
        worker.join()

    command_queue.join()

    print("\nResults:\n" + str(result_queue.qsize()))
    completed_experiments = [result_queue.get() for i in range(result_queue.qsize())]

    with open("data-latest.json", 'w') as outfile:
        json.dump([{**exp.result, **exp.configuration, 'error': exp.error} for exp in completed_experiments], outfile)

    with open("data-{:%H-%M-%d-%m-%y}.json".format(datetime.datetime.now()), 'w') as outfile:
        json.dump([{**exp.result, **exp.configuration, 'error': exp.error} for exp in completed_experiments], outfile)

    slack_notification.end_experiment_notification()


if __name__ == '__main__':
    main()
