#!/usr/bin/env python3
# Copyright (c) 2017 Intel Corporation. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import os
import sys
import subprocess
import getopt
import psutil
from multiprocessing import Process
import threading

subscription_cpp_cpu=[]
publisher_cpp_cpu=[]
subscription_node_cpu=[]
publisher_node_cpu=[]
subscription_python_cpu=[]
publisher_python_cpu=[]

subscription_cpp_memory=[]
publisher_cpp_memory=[]
subscription_node_memory=[]
publisher_node_memory=[]
subscription_python_memory=[]
publisher_python_memory=[]

average_subscription_python_cpu=0
average_publisher_python_cpu=0
average_subscription_node_cpu=0
average_publisher_node_cpu=0
average_subscription_cpp_cpu=0
average_publisher_cpp_cpu=0
average_subscription_python_memory=0
average_publisher_python_memory=0
average_subscription_node_memory=0
average_publisher_node_memory=0
average_subscription_cpp_memory=0
average_publisher_cpp_memory=0

service_cpp_cpu=[]
client_cpp_cpu=[]
service_node_cpu=[]
client_node_cpu=[]
service_python_cpu=[]
client_python_cpu=[]

service_cpp_memory=[]
client_cpp_memory=[]
service_node_memory=[]
client_node_memory=[]
service_python_memory=[]
client_python_memory=[]

average_service_python_cpu=0
average_client_python_cpu=0
average_service_node_cpu=0
average_client_node_cpu=0
average_service_cpp_cpu=0
average_client_cpp_cpu=0
average_service_python_memory=0
average_client_python_memory=0
average_service_node_memory=0
average_client_node_memory=0
average_service_cpp_memory=0
average_client_cpp_memory=0

execution_time_cpp_service_client=""
execution_time_node_service_client=""
execution_time_python_service_client=""
execution_time_cpp_subscription_publisher=""
execution_time_node_subscription_publisher=""
execution_time_python_subscription_publisher=""

def print_help():
        print(""" Python script usage
        -h, --help
        -s, --size		
        -r, --sub / pub run times
        -a, --scope= all include "C++" and "Nodejs" and "Python"
        -c, --scope= native
        -n, --scope= nodejs
        -p, --scope= python		
        -t, --service / client run times
        """)        
		
def get_prepare():
    cmd="source ../../ros2-linux/local_setup.bash;env"
    output=subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,executable="/bin/bash").communicate()[0]
    for line in output.splitlines():
        line=str(line)
        if "=" in line:
            os.environ[line.split("=")[0][2:]] = line.split("=")[1][:-1]


def monitor_subscription_start(name):
    global subscription_python_cpu,subscription_node_cpu,subscription_cpp_cpu,subscription_cpp_memory,subscription_node_memory,subscription_python_memory
    cpu_command="ps aux|grep subscription |grep -v grep|awk '{print $3}'"
    memory_command="ps aux|grep subscription |grep -v grep |awk '{print $6}'"
    subscription_cpu=[]
    subscription_memory=[]
    time.sleep(2)
    while True:
        cpu_information=os.popen(cpu_command).readline()
        if len(cpu_information)>0:
            subscription_cpu.append(str(cpu_information))
        else:
            break
        memory_information=os.popen(memory_command).readline()
        if len(memory_information)>0:
            subscription_memory.append(str(memory_information))				
        time.sleep(1)
    if name == "cpp":
        subscription_cpp_cpu=subscription_cpu[:]
        subscription_cpp_memory=subscription_memory[:]
    if name == "node":
        subscription_node_cpu=subscription_cpu[:]
        subscription_node_memory=subscription_memory[:]
    if name == "python":
        subscription_python_cpu=subscription_cpu[:]
        subscription_python_memory=subscription_memory[:]

		
def monitor_publisher_start(name):
    global publisher_python_cpu,publisher_node_cpu,publisher_cpp_cpu,publisher_cpp_memory,publisher_node_memory,publisher_python_memory
    cpu_command="ps aux|grep publisher |grep -v grep|awk '{print $3}'"
    memory_command="ps aux|grep publisher |grep -v grep| awk '{print $6}'"
    publisher_cpu=[]
    publisher_memory=[]
    time.sleep(2)
    while True:
        cpu_information=os.popen(cpu_command).readlines()
        if len(cpu_information)==2:
            publisher_cpu.append(cpu_information[1])
        else:
            break
        memory_information=os.popen(memory_command).readlines()
        if len(memory_information)==2:
            publisher_memory.append(memory_information[1])
        time.sleep(1)
    if name == "cpp":
        publisher_cpp_cpu=publisher_cpu[:]
        publisher_cpp_memory=publisher_memory[:]
    if name == "node":
        publisher_node_cpu=publisher_cpu[:]
        publisher_node_memory=publisher_memory[:]
    if name == "python":
        publisher_python_cpu=publisher_cpu[:]
        publisher_python_memory=publisher_memory[:]


def monitor_service_start(name):
    global service_python_cpu,service_node_cpu,service_cpp_cpu,service_cpp_memory,service_node_memory,service_python_memory
    cpu_command="ps aux|grep service-stress-test|grep -v grep|awk '{print $3}'"
    memory_command="ps aux|grep service-stress-test|grep -v grep |awk '{print $6}'"
    service_cpu=[]
    service_memory=[]
    time.sleep(2)
    while True:
        cpu_information=os.popen(cpu_command).readline()
        if len(cpu_information)>0:
            service_cpu.append(str(cpu_information))
        else:
            break
        memory_information=os.popen(memory_command).readline()
        if len(memory_information)>0:
            service_memory.append(str(memory_information))
        time.sleep(1)
    if name == "cpp":
        service_cpp_cpu=service_cpu[:]
        service_cpp_memory=service_memory[:]
    if name == "node":
        service_node_cpu=service_cpu[:]
        service_node_memory=service_memory[:]
    if name == "python":
        service_python_cpu=service_cpu[:]
        service_python_memory=service_memory[:]

		
def monitor_client_start(name):
    global client_python_cpu,client_node_cpu,client_cpp_cpu,client_cpp_memory,client_node_memory,client_python_memory
    cpu_command="ps aux|grep client-stress-test|grep -v grep|awk '{print $3}'"
    memory_command="ps aux|grep client-stress-test|grep -v grep |awk '{print $6}'"
    client_cpu=[]
    client_memory=[]
    time.sleep(2)
    while True:
        cpu_information=os.popen(cpu_command).readlines()
        if len(cpu_information)==2:
            client_cpu.append(cpu_information[1])
        else:
            break
        memory_information=os.popen(memory_command).readlines()
        if len(memory_information)==2:
            client_memory.append(memory_information[1])
        time.sleep(1)
    if name == "cpp":
        client_cpp_cpu=client_cpu[:]
        client_cpp_memory=client_memory[:]
    if name == "node":
        client_node_cpu=client_cpu[:]
        client_node_memory=client_memory[:]
    if name == "python":
        client_python_cpu=client_cpu[:]
        client_python_memory=client_memory[:]

def get_cpu_subscription_python():
    global subscription_python_cpu,average_subscription_python_cpu
    sum=0
    test_subscription=[]
    a=subscription_python_cpu[:]
    for i in a:
        b=i.split()
        test_subscription.append(float(b[0]))
    k=0
    for j in test_subscription:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_subscription_python_cpu=0
    else:
        average_subscription_python_cpu=sum/k
    if len(test_subscription) == 0:
        print("CPU subscription python not get CPU information")

def get_cpu_publisher_python():
    global publisher_python_cpu,average_publisher_python_cpu
    sum=0
    test_publisher=[]
    a=publisher_python_cpu[:]
    for i in a:
        b=i.split()
        test_publisher.append(float(b[0]))
    k=0
    for j in test_publisher:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_publisher_python_cpu=0
    else:
        average_publisher_python_cpu=sum/k
    if len(test_publisher) == 0:
        print("CPU publisher python not get CPU information")
	
def get_cpu_service_python():
    global service_python_cpu,average_service_python_cpu
    sum=0
    a=service_python_cpu[:]
    test_service=[]
    for i in a:
        b=i.split()
        test_service.append(float(b[0]))
    k=0
    for j in test_service:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_service_python_cpu=0
    else:
        average_service_python_cpu=sum/k
    if len(test_service) == 0:
        print("CPU service python not get CPU information")
	
def get_cpu_client_python():
    global client_python_cpu,average_client_python_cpu
    sum=0
    a=client_python_cpu[:]
    test_client=[]
    for i in a:
        b=i.split()
        test_client.append(float(b[0]))
    k=0
    for j in test_client:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_client_python_cpu=0
    else:
        average_client_python_cpu=sum/k
    if len(test_client) == 0:
        print("CPU client python not get CPU information")
	
def get_cpu_subscription_node():
    global subscription_node_cpu,average_subscription_node_cpu
    sum=0
    test_subscription=[]
    a=subscription_node_cpu[:]
    for i in a:
        b=i.split()
        test_subscription.append(float(b[0]))
    k=0
    for j in test_subscription:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_subscription_node_cpu=0
    else:
        average_subscription_node_cpu=sum/k
    if len(test_subscription) == 0:
        print("CPU subscription node not get CPU information")

def get_cpu_publisher_node():
    global publisher_node_cpu,average_publisher_node_cpu
    sum=0
    test_publisher=[]
    a=publisher_node_cpu[:]
    for i in a:
        b=i.split()
        test_publisher.append(float(b[0]))
    k=0
    for j in test_publisher:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_publisher_node_cpu=0
    else:
        average_publisher_node_cpu=sum/k
    if len(test_publisher) == 0:
        print("CPU publisher node not get CPU information")
	
def get_cpu_service_node():
    global service_node_cpu,average_service_node_cpu
    sum=0
    a=service_node_cpu[:]
    test_service=[]
    for i in a:
        b=i.split()
        test_service.append(float(b[0]))
    k=0
    for j in test_service:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_service_node_cpu=0
    else:
        average_service_node_cpu=sum/k
    if len(test_service) == 0:
        print("CPU service node not get CPU information")
	
def get_cpu_client_node():
    global client_node_cpu,average_client_node_cpu
    sum=0
    a=client_node_cpu[:]
    test_client=[]
    for i in a:
        b=i.split()
        test_client.append(float(b[0]))
    k=0
    for j in test_client:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_client_node_cpu=0
    else:
        average_client_node_cpu=sum/k
    if len(test_client) == 0:
        print("CPU client node not get CPU information")
		
def get_cpu_subscription_cpp():
    global subscription_cpp_cpu,average_subscription_cpp_cpu
    sum=0
    test_subscription=[]
    a=subscription_cpp_cpu[:]
    for i in a:
        b=i.split()
        test_subscription.append(float(b[0]))
    k=0
    for j in test_subscription:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_subscription_cpp_cpu=0
    else:
        average_subscription_cpp_cpu=sum/k
    if len(test_subscription) == 0:
        print("CPU subscription c++ not get CPU information")

def get_cpu_publisher_cpp():
    global publisher_cpp_cpu,average_publisher_cpp_cpu
    sum=0
    test_publisher=[]
    a=publisher_cpp_cpu[:]
    for i in a:
        b=i.split()
        test_publisher.append(float(b[0]))
    k=0
    for j in test_publisher:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_publisher_cpp_cpu=0
    else:
        average_publisher_cpp_cpu=sum/k
    if len(test_publisher) == 0:
        print("CPU publisher c++ not get CPU information")
	
def get_cpu_service_cpp():
    global service_cpp_cpu,average_service_cpp_cpu
    sum=0
    a=service_cpp_cpu[:]
    test_service=[]
    for i in a:
        b=i.split()
        test_service.append(float(b[0]))
    k=0
    for j in test_service:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_service_cpp_cpu=0
    else:
        average_service_cpp_cpu=sum/k
    if len(test_service) == 0:
        print("CPU service c++ not get CPU information")
	
def get_cpu_client_cpp():
    global client_cpp_cpu,average_client_cpp_cpu
    sum=0
    a=client_cpp_cpu[:]
    test_client=[]
    for i in a:
        b=i.split()
        test_client.append(float(b[0]))
    k=0
    for j in test_client:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_client_cpp_cpu=0
    else:
        average_client_cpp_cpu=sum/k
    if len(test_client) == 0:
        print("CPU client c++ not get CPU information")

def get_memory_subscription_python():
    global subscription_python_memory,average_subscription_python_memory
    sum=0
    a=subscription_python_memory[:]
    test_subscription=[]
    for i in a:
        b=i.split()
        test_subscription.append(float(b[0]))
    k=0
    for j in test_subscription:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_subscription_python_memory=0
    else:
        average_subscription_python_memory=sum/k/1024
    if len(test_subscription) == 0:
        print("Memory subscription python not get Memory information")	
		
def get_memory_publisher_python():
    global publisher_python_memory,average_publisher_python_memory
    sum=0
    test_publisher=[]
    a=publisher_python_memory[:]
    for i in a:
        b=i.split()
        test_publisher.append(float(b[0]))
    k=0
    for j in test_publisher:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_publisher_python_memory=0
    else:
        average_publisher_python_memory=sum/k/1024
    if len(test_publisher) == 0:
        print("Memory publisher python not get Memory information")
		
def get_memory_service_python():
    global service_python_memory,average_service_python_memory
    sum=0
    a=service_python_memory[:]
    test_service=[]
    for i in a:
        b=i.split()
        test_service.append(float(b[0]))
    k=0
    for j in test_service:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_service_python_memory=0
    else:
        average_service_python_memory=sum/k/1024
    if len(test_service) == 0:
        print("Memory service python not get Memory information")
		
def get_memory_client_python():
    global client_python_memory,average_client_python_memory
    sum=0
    a=client_python_memory[:]
    test_client=[]
    for i in a:
        b=i.split()
        test_client.append(float(b[0]))
    k=0
    for j in test_client:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_client_python_memory=0
    else:
        average_client_python_memory=sum/k/1024
    if len(test_client) == 0:
        print("Memory client python not get Memory information")

def get_memory_subscription_node():
    global subscription_node_memory,average_subscription_node_memory
    sum=0
    a=subscription_node_memory[:]
    test_subscription=[]
    for i in a:
        b=i.split()
        test_subscription.append(float(b[0]))
    k=0
    for j in test_subscription:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_subscription_node_memory=0
    else:
        average_subscription_node_memory=sum/k/1024
    if len(test_subscription) == 0:
        print("Memory subscription node not get Memory information")	
		
def get_memory_publisher_node():
    global publisher_node_memory,average_publisher_node_memory
    sum=0
    test_publisher=[]
    a=publisher_node_memory[:]
    for i in a:
        b=i.split()
        test_publisher.append(float(b[0]))
    k=0
    for j in test_publisher:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_publisher_node_memory=0
    else:
        average_publisher_node_memory=sum/k/1024
    if len(test_publisher) == 0:
        print("Memory publisher node not get Memory information")
		
def get_memory_service_node():
    global service_node_memory,average_service_node_memory
    sum=0
    a=service_node_memory[:]
    test_service=[]
    for i in a:
        b=i.split()
        test_service.append(float(b[0]))
    k=0
    for j in test_service:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_service_node_memory=0
    else:
        average_service_node_memory=sum/k/1024
    if len(test_service) == 0:
        print("Memory service node not get Memory information")
		
def get_memory_client_node():
    global client_node_memory,average_client_node_memory
    sum=0
    a=client_node_memory[:]
    test_client=[]
    for i in a:
        b=i.split()
        test_client.append(float(b[0]))
    k=0
    for j in test_client:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_client_node_memory=0
    else:
        average_client_node_memory=sum/k/1024
    if len(test_client) == 0:
        print("Memory client node not get Memory information")

def get_memory_subscription_cpp():
    global subscription_cpp_memory,average_subscription_cpp_memory
    sum=0
    test_subscription=[]
    a=subscription_cpp_memory[:]
    for i in a:
        b=i.split()
        test_subscription.append(float(b[0]))
    k=0
    for j in test_subscription:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_subscription_cpp_memory=0
    else:
        average_subscription_cpp_memory=sum/k/1024
    if len(test_subscription) == 0:
        print("Memory subscription c++ not get Memory information")
		
def get_memory_publisher_cpp():
    global publisher_cpp_memory,average_publisher_cpp_memory
    sum=0
    test_publisher=[]
    a=publisher_cpp_memory[:]
    for i in a:
        b=i.split()
        test_publisher.append(float(b[0]))
    k=0
    for j in test_publisher:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_publisher_cpp_memory=0
    else:
        average_publisher_cpp_memory=sum/k/1024
    if len(test_publisher) == 0:
        print("Memory publisher c++ not get Memory information")
		
def get_memory_service_cpp():
    global service_cpp_memory,average_service_cpp_memory
    sum=0
    a=service_cpp_memory[:]
    test_service=[]
    for i in a:
        b=i.split()
        test_service.append(float(b[0]))
    k=0
    for j in test_service:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_service_cpp_memory=0
    else:
        average_service_cpp_memory=sum/k/1024
    if len(test_service) == 0:
        print("Memory service c++ not get Memory information")
		
def get_memory_client_cpp():
    global client_cpp_memory,average_client_cpp_memory
    sum=0
    a=client_cpp_memory[:]
    test_client=[]
    for i in a:
        b=i.split()
        test_client.append(float(b[0]))
    k=0
    for j in test_client:
        if j==0 and k==0:
            continue
        sum=sum+float(j)
        k+=1
    if k==0:
        average_client_cpp_memory=0
    else:
        average_client_cpp_memory=sum/k/1024
    if len(test_client) == 0:
        print("Memory client c++ not get Memory information")

def get_execution_time_cpp_service_client(result):
    global execution_time_cpp_service_client
    execution_time=0
    plist=result.split()
    for number in range(len(plist)):
        if plist[number] == "seconds":
            execution_time=execution_time+int(plist[number-1])*1000
        if plist[number] == "milliseconds.":
            execution_time=execution_time+int(plist[number-1])
    execution_time_cpp_service_client=str(execution_time)
	
def get_execution_time_node_service_client(result):
    global execution_time_node_service_client
    execution_time=0
    plist=result.split()
    for number in range(len(plist)):
        if plist[number] == "seconds":
            execution_time=execution_time+int(plist[number-1])*1000
        if plist[number] == "milliseconds.":
            execution_time=execution_time+int(plist[number-1])
    execution_time_node_service_client=str(execution_time)

def get_execution_time_python_service_client(result):
    global execution_time_python_service_client
    execution_time=0
    plist=result.split()
    for number in range(len(plist)):
        if plist[number] == "seconds":
            execution_time=execution_time+int(plist[number-1])*1000
        if plist[number] == "milliseconds.":
            execution_time=execution_time+int(plist[number-1])
    execution_time_python_service_client=str(execution_time)

def get_execution_time_cpp_subscription_publisher(result):
    global execution_time_cpp_subscription_publisher
    execution_time=0
    plist=result.split()
    for number in range(len(plist)):
        if plist[number] == "seconds":
            execution_time=execution_time+int(plist[number-1])*1000
        if plist[number] == "milliseconds.":
            execution_time=execution_time+int(plist[number-1])
    execution_time_cpp_subscription_publisher=str(execution_time)

def get_execution_time_node_subscription_publisher(result):
    global execution_time_node_subscription_publisher
    execution_time=0
    plist=result.split()
    for number in range(len(plist)):
        if plist[number] == "seconds":
            execution_time=execution_time+int(plist[number-1])*1000
        if plist[number] == "milliseconds.":
            execution_time=execution_time+int(plist[number-1])
    execution_time_node_subscription_publisher=str(execution_time)

def get_execution_time_python_subscription_publisher(result):
    global execution_time_python_subscription_publisher
    execution_time=0
    plist=result.split()
    for number in range(len(plist)):
        if plist[number] == "seconds":
            execution_time=execution_time+int(plist[number-1])*1000
        if plist[number] == "milliseconds.":
            execution_time=execution_time+int(plist[number-1])
    execution_time_python_subscription_publisher=str(execution_time)

def kill_subscription_publisher_process():
    command="ps aux|grep subscription |awk '{print $2}'"
    plist=os.popen(command).readlines()
    for pid in plist:
        if psutil.pid_exists(int(pid)):
            psutil.Process(int(pid)).kill()
    command="ps aux|grep publisher |awk '{print $2}'"
    plist=os.popen(command).readlines()
    for pid in plist:
        if psutil.pid_exists(int(pid)):
            psutil.Process(int(pid)).kill()

def kill_service_client_process():
    command="ps aux|grep service-stress-test|awk '{print $2}'"
    plist=os.popen(command).readlines()
    for pid in plist:
        if psutil.pid_exists(int(pid)):
            psutil.Process(int(pid)).kill()
    command="ps aux|grep client-stress-test|awk '{print $2}'"
    plist=os.popen(command).readlines()
    for pid in plist:
        if psutil.pid_exists(int(pid)):
            psutil.Process(int(pid)).kill()			
			
def kill_all_process():
    command="ps aux|grep subscription |awk '{print $2}'"
    plist=os.popen(command).readlines()
    for pid in plist:
        if psutil.pid_exists(int(pid)):
            psutil.Process(int(pid)).kill()
    command="ps aux|grep publisher |awk '{print $2}'"
    plist=os.popen(command).readlines()
    for pid in plist:
        if psutil.pid_exists(int(pid)):
            psutil.Process(int(pid)).kill()

    command="ps aux|grep service-stress-test|awk '{print $2}'"
    plist=os.popen(command).readlines()
    for pid in plist:
        if psutil.pid_exists(int(pid)):
            psutil.Process(int(pid)).kill()
    command="ps aux|grep client-stress-test|awk '{print $2}'"
    plist=os.popen(command).readlines()
    for pid in plist:
        if psutil.pid_exists(int(pid)):
            psutil.Process(int(pid)).kill()

def cpp_subscription_publisher_test():
    global scope,size,run,run_sc
    get_prepare()
    os.chdir("./rclcpp")
    result=os.popen("colcon build")
    os.chdir("./build/rclcpp_benchmark")
    monitor_c_subscription = threading.Thread(target=monitor_subscription_start, args=('cpp',), name='monitor_subscription')
    monitor_c_publisher = threading.Thread(target=monitor_publisher_start, args=('cpp',), name='monitor_publisher')
    monitor_c_subscription.start()
    monitor_c_publisher.start()
    os.system("./subscription-stress-test&")
    cmd="./publisher-stress-test --run="+run+" --size="+size
    time.sleep(1)
    result=os.popen(cmd).readlines()
    time.sleep(1)
    kill_subscription_publisher_process()
    time.sleep(2)
    get_cpu_subscription_cpp()
    get_cpu_publisher_cpp()
    get_memory_subscription_cpp()
    get_memory_publisher_cpp()
    get_execution_time_cpp_subscription_publisher(result[-1])    
    os.chdir("../../..")

def cpp_service_client_test():
    global scope,size,run,run_sc
    get_prepare()
    os.chdir("./rclcpp")
    result=os.popen("colcon build")
    os.chdir("./build/rclcpp_benchmark")
    monitor_c_service = threading.Thread(target=monitor_service_start, args=('cpp',), name='monitor_service')
    monitor_c_client = threading.Thread(target=monitor_client_start, args=('cpp',), name='monitor_client')
    monitor_c_service.start()
    monitor_c_client.start()
    cmd="./service-stress-test --size="+size+"&"
    os.system(cmd)
    cmd="./client-stress-test --run="+run_sc
    time.sleep(1)
    result=os.popen(cmd).readlines()
    time.sleep(1)
    kill_service_client_process()
    time.sleep(2)
    get_cpu_service_cpp()
    get_cpu_client_cpp()
    get_memory_service_cpp()
    get_memory_client_cpp()
    get_execution_time_cpp_service_client(result[-1])	
    os.chdir("../../..")

def node_subscription_publisher_test():
    global scope,size,run,run_sc
    get_prepare()
    os.chdir("./rclnodejs/topic")
    monitor_n_subscription = threading.Thread(target=monitor_subscription_start, args=('node',), name='monitor_subscription')
    monitor_n_publisher = threading.Thread(target=monitor_publisher_start, args=('node',), name='monitor_publisher')
    monitor_n_subscription.start()
    monitor_n_publisher.start()
    os.system("node subscription-stress-test.js&")
    cmd="node publisher-stress-test.js -r "+run+" -s "+size
    time.sleep(1)
    result=os.popen(cmd).readlines()
    time.sleep(1)
    kill_subscription_publisher_process()
    time.sleep(2)
    get_cpu_subscription_node()
    get_cpu_publisher_node()
    get_memory_subscription_node()
    get_memory_publisher_node()
    get_execution_time_node_subscription_publisher(result[-1])	
    os.chdir("../..")

def node_service_client_test():	
    global scope,size,run,run_sc
    get_prepare()	
    os.chdir("./rclnodejs/service")
    monitor_n_service = threading.Thread(target=monitor_service_start, args=('node',), name='monitor_service')
    monitor_n_client = threading.Thread(target=monitor_client_start, args=('node',), name='monitor_client')
    monitor_n_service.start()
    monitor_n_client.start()
    cmd="node service-stress-test.js -s "+size+"&"
    os.system(cmd)
    cmd="node client-stress-test.js -r "+run_sc
    time.sleep(1)
    result=os.popen(cmd).readlines()
    time.sleep(1)
    kill_service_client_process()
    time.sleep(2)
    get_cpu_service_node()
    get_cpu_client_node()
    get_memory_service_node()
    get_memory_client_node()
    get_execution_time_node_service_client(result[-1])	
    os.chdir("../..")
	
def python_subscription_publisher_test():
    global scope,size,run,run_sc
    get_prepare()
    os.chdir("./rclpy/topic")
    monitor_p_subscription = threading.Thread(target=monitor_subscription_start, args=('python',), name='monitor_subscription')
    monitor_p_publisher = threading.Thread(target=monitor_publisher_start, args=('python',), name='monitor_publisher')
    monitor_p_subscription.start()
    monitor_p_publisher.start()
    os.system("python3 subscription-stress-test.py&")
    cmd="python3 publisher-stress-test.py -r "+run+" -s "+size
    time.sleep(1)
    result=os.popen(cmd).readlines()
    time.sleep(1)
    kill_subscription_publisher_process()
    time.sleep(2)
    get_cpu_subscription_python()
    get_cpu_publisher_python()
    get_memory_subscription_python()
    get_memory_publisher_python()	
    get_execution_time_python_subscription_publisher(result[-1])
    os.chdir("../..")	

def python_service_client_test():
    global scope,size,run,run_sc
    get_prepare()	
    os.chdir("./rclpy/service")
    monitor_p_service = threading.Thread(target=monitor_service_start, args=('python',), name='monitor_service')
    monitor_p_client = threading.Thread(target=monitor_client_start, args=('python',), name='monitor_client')
    monitor_p_service.start()
    monitor_p_client.start()
    cmd="python3 service-stress-test.py -s "+size+"&"
    os.system(cmd)
    cmd="python3 client-stress-test.py -r "+run_sc
    time.sleep(1)
    result=os.popen(cmd).readlines()
    time.sleep(1)
    kill_service_client_process()
    time.sleep(2)
    get_cpu_service_python()
    get_cpu_client_python()
    get_memory_service_python()
    get_memory_client_python()
    get_execution_time_python_service_client(result[-1])
    os.chdir("../..")	

def get_record():
    global scope,size,run,run_sc
    name=time.strftime("%Y%m%d-%H:%M:%S.txt", time.localtime())
    msg = ""
    file_handle = open(name, 'w')

    msg+="Summary:\n"
    msg+="Benchmark Testing against ROS2 Sample(subscription/publisher):\n"
    msg+="size is: "+size+"KB, cycle is: "+run+"\n"
    if scope=="all" or scope == "native":	
        msg+="Sample type: CPP\n"
        msg+="CPU:"+"%.2f" %(average_subscription_cpp_cpu+average_publisher_cpp_cpu)+"%\n"
        msg+="Memory:"+"%.2f" %(average_subscription_cpp_memory+average_publisher_cpp_memory)+" MB\n"
        msg+="Execution time:"+execution_time_cpp_subscription_publisher+" millisecond\n"
	
    if scope=="all" or scope == "nodejs":	
        msg+="Sample type: Nodejs\n"
        msg+="CPU: "+"%.2f" %(average_subscription_node_cpu+average_publisher_node_cpu)+"%\n"
        msg+="Memory: "+"%.2f" %(average_subscription_node_memory+average_publisher_node_memory)+" MB\n"
        msg+="Execution time: "+execution_time_node_subscription_publisher+" millisecond\n"
	
    if scope=="all" or scope == "python":	
        msg+="Sample type: Python\n"
        msg+="CPU: "+"%.2f" %(average_subscription_python_cpu+average_publisher_python_cpu)+"%\n"
        msg+="Memory: "+"%.2f" %(average_subscription_python_memory+average_publisher_python_memory)+" MB\n"
        msg+="Execution time: "+execution_time_python_subscription_publisher+" millisecond\n"
	
    msg+="\nBenchmarkBenchmark Testing against ROS2 Sample(service/client):\n"
    msg+="size is: "+size+"KB, cycle is: "+run_sc+"\n"
    if scope=="all" or scope == "native":
        msg+="Sample type: CPP\n"
        msg+="CPU: "+"%.2f" %(average_service_cpp_cpu+average_client_cpp_cpu)+"%\n"
        msg+="Memory: "+"%.2f" %(average_service_cpp_memory+average_client_cpp_memory)+" MB\n"
        msg+="Execution time: "+execution_time_cpp_service_client+" millisecond\n"
	
    if scope=="all" or scope == "nodejs":
        msg+="Sample type: Nodejs\n"
        msg+="CPU:"+"%.2f" %(average_service_node_cpu+average_client_node_cpu)+"%\n"
        msg+="Memory:"+"%.2f" %(average_service_node_memory+average_client_node_memory)+" MB\n"
        msg+="Execution time:"+execution_time_node_service_client+" millisecond\n"
	
    if scope=="all" or scope == "python":
        msg+="Sample type: Python\n"
        msg+="CPU: "+"%.2f" %(average_service_python_cpu+average_client_python_cpu)+"%\n"
        msg+="Memory: "+"%.2f" %(average_service_python_memory+average_client_python_memory)+" MB\n"
        msg+="Execution time: "+execution_time_python_service_client+" millisecond\n"
	
    file_handle.write(msg)
    file_handle.close()
    print("Benchmark finished! The record file path: "+sys.path[0]+"/"+name)
	
def main():
    global scope,size,run,run_sc 
    kill_all_process()
    opts, args = getopt.getopt(sys.argv[1:], "acnpht:r:s:")
	
    scope="all"
    size="1000"
    run="20000"
    run_sc="10000"
	
    for op, value in opts:
        if op == "-r":
            run=value
        elif op == "-s":
            size=value
        elif op == "-t":
            run_sc=value
        elif op == "-a":
            scope="all"
        elif op == "-c":
            scope="native"
        elif op == "-n":
            scope="nodejs"
        elif op == "-p":
            scope="python"
        elif op == "-h":
            print_help()
            sys.exit()
		
    if scope=="all":
        print("Benchmark test for all samples:\nTest size: "+size+"KB\nTest run: "+run+" times(for subscription/publisher sample)\nTest run: "+run_sc+" times(for service/client sample)\nBegining......")
        cpp_subscription_publisher_test()
        cpp_service_client_test()
        node_subscription_publisher_test()
        node_service_client_test()
        python_subscription_publisher_test()
        python_service_client_test()
    elif scope=="native":
        print("Benchmark test only for native:\nTest size: "+size+"KB\nTest run: "+run+" times(for subscription/publisher sample)\nTest run: "+run_sc+" times(for service/client sample)\nBegining......")
        cpp_subscription_publisher_test()
        cpp_service_client_test()
    elif scope=="nodejs":
        print("Benchmark test only for nodejs:\nTest size: "+size+"KB\nTest run: "+run+" times(for subscription/publisher sample)\nTest run: "+run_sc+" times(for service/client sample)\nBegining......")
        node_subscription_publisher_test()   
        node_service_client_test()
    elif scope == "python":
        print("Benchmark test only for python:\nTest size: "+size+"KB\nTest run: "+run+" times(for subscription/publisher sample)\nTest run: "+run_sc+" times(for service/client sample)\nBegining......")
        python_subscription_publisher_test()
        python_service_client_test()
    get_record()
		
if __name__ == "__main__":
    main()    