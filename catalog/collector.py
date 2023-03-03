import subprocess
import time
import json 

from domains import DOMAIN_COMPONENTS

def run_command(command):
    output = subprocess.Popen(command, shell=True,
                              stdout=subprocess.PIPE).stdout.read().decode('utf-8').strip().split('\n')
    return output

def _parse_node_output(node) :
    topics = []
    actions = []
    services = []
    _output = run_command(f'ros2 node info {node}')
    method = None
    for line in _output:
        if "Subscribers:" in line or "Publishers:" in line:
            method = topics
        elif "Service Servers:" in line or "Service Clients:" in line:
            method = services
        elif "Action Servers:" in line or "Action Clients:" in line:
            method = actions
        else:
            if len(line) > 0 and line != node:
                name, type = line.strip().split(': ')
                method.append((name, type))
    return topics, services, actions



class DataCollector:
    def __init__(self):
        self._data = {
            "domain": {},
            "node": {},
            "topic": {},
            "type": {}
        }
        self.node_domain_mapping = {}

    def collect(self):
        self._domain_data()
        self._sensor_data()
        return self._data

    def _type_info(self, _type):
        if _type not in self._data['type']:
            type_info = run_command('ros2 interface show ' + _type)
            self._data['type'][_type] = {
                "entity": {
                    "typeName": "type",
                    "attributes": {
                        "name": _type,
                        "info": type_info,
                        "create_time": time.time()
                    }
                }
            }

    def _domain_data(self) :
        for key in DOMAIN_COMPONENTS :
            for node in DOMAIN_COMPONENTS[key] :
                self.node_domain_mapping[node] = key
            self._data["domain"][key] = {
                "entity": {
                    "typeName": "domain",
                    "attributes": {
                        "name": key,
                        "create_time": time.time()
                    }
                }
            }

    def _node_data(self) :
        nodes = run_command('ros2 node list')
        for node in nodes :
            node_info = {
                "name":node,
                "create_time": time.time(),
            }
            topics, services, actions = _parse_node_output(node)
            self._topic_data(node, topics)
            # self._service_info(node, services)
            # self._action_info(node, actions)
    
    def _topic_data(self, node, topics):
        for topic, _type in topics:
            self._type_info(_type)

            output_lines = run_command('ros2 topic info ' + topic)
            topic_info = {
                "name": topic,
                "create_time": time.time()
            }
            _type = ""
            for line in output_lines:
                if "Publisher count" in line:
                    topic_info["publisher_count"] = line.split(": ")[1]
                elif "Subscription count" in line:
                    topic_info["subscriber_count"] = line.split(": ")[1]
            if topic not in self._data['topic']:
                self._data['topic'][topic] = {"entity": {
                    "typeName": "topic",
                    "attributes": topic_info,
                    "relationshipAttributes": {
                        "node": {"typeName": "node", "uniqueAttributes": {"name": node}},
                        "type": {"typeName": "type", "uniqueAttributes": {"name": _type}}
                    }
                }}

    def _service_info(self, node, services):
        for service, _type in services:
            self._type_info(_type)
            if service not in self._data['service']:
                self._data['service'][service] = {"entity": {
                    "typeName": "service",
                    "attributes": {"name": f'{node_type[:-1]}_{name}', "create_time": time.time()},
                    "relationshipAttributes": {
                        "node": {"typeName": "node", "uniqueAttributes": {"name": node}},
                        "type": {"typeName": "type", "uniqueAttributes": {"name": _type}}
                    }
                }}

    def _action_info(self, node, actions):
        for itr, node_type in actions:
            action, _type = itr.strip(" ").split(': ')
            _, node, name = _strip_name(action)
            _output = run_command('ros2 action info ' + action)
            self._type_info(_type)
            action_info = {"name": f'{node_type[:-1]}_{name}', "create_time": time.time()}
            for line in _output:
                if "Action clients" in line:
                    action_info["action_clients"] = line.split(": ")[1]
                if "Action servers" in line:
                    action_info["action_servers"] = line.split(": ")[1]

            if action not in self._data['action']:
                self._data['action'][action] = {"entity": {
                    "typeName": "action",
                    "attributes": action_info,
                    "relationshipAttributes": {
                        "node": {"typeName": "node", "uniqueAttributes": {"name": node}},
                        "type": {"typeName": "type", "uniqueAttributes": {"name": _type}}
                    }
                }}