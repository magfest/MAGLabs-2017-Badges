#!/usr/bin/python
import multiprocessing
import sys
import os

with open("./dhcpd.leases", "r") as FILE:
  data = FILE.read()

ips = []

for i in data.split("lease"):
  if "ESP_" in i:
    ip = i.split(" ")[1]
    ips.append(ip)
    print(ip)

ips = list(set(ips))
print("Found {} hosts that match.".format(len(ips)))

def testip(ip):
  return not os.system("ping -w 1 -i 0.2 -c 3 {}".format(ip))

alive = []
for i in ips:
  print("Trying {}".format(i))
  if not os.system("ping -w 1 -i 0.2 -c 3 {}".format(i)):
    print("  Host found alive.")
    alive.append(i)
    os.system("make netburn IP={}".format(i))
  else:
    print("  Host found dead.")

print("Found {} alive hosts.".format(len(alive)))
print(alive)
