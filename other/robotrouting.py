#!/usr/bin/python3
import pyroute2
import time, threading
import yaml

class RobotRouting:
    def __init__(self, graph, iface='wlan1'):
        self.original_graph = graph
        self.iface = iface
        self.ip_data = self.readConfig('data.yaml')
        self.robot_to_id = {}
        for id_ in self.ip_data:
            self.robot_to_id[self.ip_data[id_]['id']] = id_

    def getRoute(self, from_id):
        from_ip = self.ip_data[self.robot_to_id[from_id]]['ip']
        routing_str = f'sshpass -p verlabpi ssh ubuntu@{from_ip} -t \''
        for id_ in self.original_graph:
           if(from_id != id_):
               path = self.getPath(from_id, id_)
               ip_addr_to = self.ip_data[self.robot_to_id[id_]]['ip']
               ip_addr_next = self.ip_data[self.robot_to_id[path[1]]]['ip']
               #print(ip_addr_from, ip_addr_next)
               routing_str += f'sudo ip route add {ip_addr_to} via {ip_addr_next} dev {self.iface} & '
               #print(f'{self.ip_data[from_id]['ip']} : nexthop {id_} {path[1]}')    

        routing_str = routing_str[:-2] + '\''
        print(routing_str)

        return ''

    def getPath(self, from_id, to_id):
        
        self.graph = dict(self.original_graph) #{1: [2], 2:[1,3,5], 3:[2,4], 4:[3], 5:[2]}
        #print('graph', self.graph)
        path, found = self.getPathIteractive(from_id, to_id, visited=set([]))
        if(path == []):
            return None
            
        path.append(from_id)
        return path[::-1]

    def getPathIteractive(self, from_id, to_id, visited=set([])):

        visited.add(from_id)
        for next_hop in self.graph[from_id]:
            if next_hop in visited:
                continue

            if(to_id == next_hop):
                return [next_hop], True  

            path, found = self.getPathIteractive(next_hop, to_id, visited)

            if(found):
                path.append(next_hop)      
                return path, True

        return [], False


    def readConfig(self, config_file):
        with open(config_file, 'r') as stream:
            return yaml.safe_load(stream)['robots']

if __name__ == "__main__":
    #1 20 21 2
    #1 20 22 3

    graph = {0: [20], 20:[0,21, 22], 21:[1, 20], 22:[20, 2], 1:[21], 2:[22]}
    routing = RobotRouting(graph)
    
    path_1 = routing.getPath(0, 1)
    path_2 = routing.getPath(0, 2)


    for i in range(len(path_1)):
        if(path_1[i] != path_2[i]):
            break

    path_1.extend(path_2[i:])
    

    for id_ in path_1:
        print(f'\n\n\n IP {id_} \n ')
        print(routing.getRoute(id_))
    exit(1)
