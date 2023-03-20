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
            "type": {},
            "action": {},
            "service": {}
        }

    def get_data(self):
        self._domain_data()
        self._node_info()
        return self._data
    
    def _domain_data(self) :
        self._data["domain"]["default_domain"] = {
            "entity": {
                "typeName": "domain",
                "attributes": {
                    "name": "default_domain",
                    "create_time": time.time()
                }
            }
        }


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

    def _node_info(self):
        nodes = set(run_command('ros2 node list'))
        nodes.add("default_node")
        node_info = {
            "topics": set(),
            "actions": set(),
            "services": set()
        }
        for node in nodes:
            if node not in self._data['node']:
                self._data['node'][node] = {"entity": {
                    "typeName": "node",
                    "attributes": {"name": node, "create_time": time.time()},
                    "relationshipAttributes": {
                        "domain": {"typeName": "domain", "uniqueAttributes": {"name": "default_domain"}}
                    }
                }}

            _output = run_command("ros2 node info " + node)
            key = ""
            _type = ""
            for line in _output:
                if "Subscribers:" in line or "Publishers:" in line:
                    key = "topics"
                    _type = line.replace(':', '').strip().replace(' ', '_').lower()
                elif "Service Servers:" in line or "Service Clients:" in line:
                    key = "services"
                    _type = line.replace(':', '').strip().replace(' ', '_').lower()
                elif "Action Servers:" in line or "Action Clients:" in line:
                    key = "actions"
                    _type = line.replace(':', '').strip().replace(' ', '_').lower()
                else:
                    if len(line) > 0 and line != node:
                        node_info[key].add((line, _type))
        self._topic_info(node_info["topics"], node)
        self._action_info(node_info["actions"], node)
        self._service_info(node_info["services"], node)

    def _topic_info(self, topics, node):
        for itr, node_type in topics:
            topic, _type = itr.strip(" ").split(': ')

            output_lines = run_command('ros2 topic info ' + topic)
            node_type = 'publishing' if node_type == 'publishers' else 'subscribing'
            topic_info = {
                "name": f'{node_type}_{topic}',
                "create_time": time.time()
            }
            self._type_info(_type)
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
                        "ecu": {"typeName": "ecu", "uniqueAttributes": {"name": node}},
                        "type": {"typeName": "type", "uniqueAttributes": {"name": _type}}
                    }
                }}

    def _service_info(self, services, node):
        for itr, node_type in services:
            service, _type = itr.strip(" ").split(': ')
            self._type_info(_type)
            if service not in self._data['service']:
                self._data['service'][service] = {"entity": {
                    "typeName": "service",
                    "attributes": {"name": f'{node_type[:-1]}_{service}', "create_time": time.time()},
                    "relationshipAttributes": {
                        "ecu": {"typeName": "ecu", "uniqueAttributes": {"name": node}},
                        "type": {"typeName": "type", "uniqueAttributes": {"name": _type}}
                    }
                }}

    def _action_info(self, actions, node):
        for itr, node_type in actions:
            action, _type = itr.strip(" ").split(': ')
            _output = run_command('ros2 action info ' + action)
            self._type_info(_type)
            action_info = {"name": f'{node_type[:-1]}_{action}', "create_time": time.time()}
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
                        "ecu": {"typeName": "ecu", "uniqueAttributes": {"name": node}},
                        "type": {"typeName": "type", "uniqueAttributes": {"name": _type}}
                    }
                }}